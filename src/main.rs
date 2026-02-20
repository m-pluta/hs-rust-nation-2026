use anyhow::{bail, Context, Result};
use log::{debug, error, info, warn};
use opencv::{
    aruco::{
        self, get_predefined_dictionary, DetectorParameters, Dictionary, PREDEFINED_DICTIONARY_NAME,
    },
    core::{Point2f, Vector},
    imgcodecs,
    prelude::*,
};
use reqwest::blocking::Client;
use serde::Serialize;
use std::{
    collections::HashMap,
    f64::consts::PI,
    thread,
    time::{Duration, Instant},
};

// ─── Configuration ────────────────────────────────────────────────────────────

const CAR_URL: &str = "http://hackathon-9-car.local:5000";
const CAR_AUTH: &str = "374744";

const CAM1_URL: &str = "http://hackathon-11-camera.local:50051/frame";
const CAM1_AUTH: &str = "983149";

const CAM2_URL: &str = "http://hackathon-12-camera.local:50051/frame";
const CAM2_AUTH: &str = "378031";

const ORACLE_URL: &str = "http://192.168.0.56:31415/quadrant";
const ORACLE_AUTH: &str = "606545";

const CAR_MARKER_ID: i32 = 9;

/// Heading error (rad) below which we drive forward instead of spinning.
const ANGLE_OK: f64 = 0.50;

/// Flip to -1.0 if the car turns the wrong direction when angle_err > 0.
const TURN_POLARITY: f32 = -1.0;

// ─── Types ────────────────────────────────────────────────────────────────────

#[derive(Serialize)]
struct DriveCmd {
    speed: f32,
    flip: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Quadrant {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
}

impl Quadrant {
    fn from_pos(x: f64, y: f64, w: f64, h: f64) -> Self {
        match (x > w / 2.0, y > h / 2.0) {
            (false, false) => Self::TopLeft,
            (true, false) => Self::TopRight,
            (false, true) => Self::BottomLeft,
            (true, true) => Self::BottomRight,
        }
    }

    fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_uppercase().as_str() {
            "13" | "TL" | "Q1" | "1" | "TOP_LEFT" => Some(Self::TopLeft),
            "11" | "TR" | "Q2" | "2" | "TOP_RIGHT" => Some(Self::TopRight),
            "14" | "BL" | "Q3" | "3" | "BOTTOM_LEFT" => Some(Self::BottomLeft),
            "12" | "BR" | "Q4" | "4" | "BOTTOM_RIGHT" => Some(Self::BottomRight),
            _ => None,
        }
    }
}

// ─── Main loop ────────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    info!("Car #{CAR_MARKER_ID} — starting up");
    info!("  car     : {CAR_URL}");
    info!("  camera1 : {CAM1_URL}");
    info!("  camera2 : {CAM2_URL}");
    info!("  oracle  : {ORACLE_URL}");

    let client = Client::builder()
        .timeout(Duration::from_secs(5))
        .build()
        .context("HTTP client")?;

    let detector = make_detector()?;
    info!("ArUco detector ready (DICT_4X4_50, car marker id={CAR_MARKER_ID})");

    let mut tl = None;
    let mut tr = None;
    let mut bl = None;
    let mut br = None;

    let mut target: Option<Quadrant> = None;
    let mut last_oracle = Instant::now() - Duration::from_secs(30);
    let mut no_car_count = 0u32;

    loop {
        // ── Oracle poll ──────────────────────────────────────────────────────
        if last_oracle.elapsed() >= Duration::from_secs(2) {
            match query_oracle(&client) {
                Ok(q) => {
                    if target != Some(q) {
                        info!("🎯 New target quadrant: {:?}", q);
                        target = Some(q);
                    } else {
                        debug!("Oracle: still {:?}", q);
                    }
                }
                Err(e) => error!("Oracle poll failed: {e}"),
            }
            last_oracle = Instant::now();
        }

        let Some(tgt) = target else {
            debug!("Waiting for first oracle response…");
            thread::sleep(Duration::from_millis(200));
            continue;
        };

        // ── Camera frame ─────────────────────────────────────────────────────
        let frame1 = match fetch_frame(&client, CAM1_URL, CAM1_AUTH) {
            Ok(f) => {
                debug!("Frame from camera1 ({}×{})", f.cols(), f.rows());
                Some(f)
            }
            Err(e) => {
                error!("camera 1 failed: {e}");
                None
            }
        };
        let frame2 = match fetch_frame(&client, CAM2_URL, CAM2_AUTH) {
            Ok(f) => {
                debug!("Frame from camera2 ({}×{})", f.cols(), f.rows());
                Some(f)
            }
            Err(e) => {
                error!("camera 2 failed: {e}");
                None
            }
        };

        let mut car = None;
        let mut w = None;
        let mut h = None;
        for frame in [frame1, frame2].into_iter().flatten() {
            // ── Detect our car ───────────────────────────────────────────────────
            let items = match detect_car(&detector, &frame) {
                Err(e) => {
                    error!("Detection error: {e}");
                    thread::sleep(Duration::from_millis(100));
                    continue;
                }
                Ok(items) => items,
            };

            let frame_car = items.get(&CAR_MARKER_ID).copied();

            let mut found = false;

            if let Some(&pos) = items.get(&13) {
                debug!("found pos of TopLeft: {pos:?}");
                tl = Some(pos);

                if tgt == Quadrant::TopLeft {
                    found = true;
                }
            }
            if let Some(&pos) = items.get(&11) {
                debug!("found pos of TopRight: {pos:?}");
                tr = Some(pos);

                if tgt == Quadrant::TopRight {
                    found = true;
                }
            }
            if let Some(&pos) = items.get(&14) {
                debug!("found pos of BottomLeft: {pos:?}");
                bl = Some(pos);

                if tgt == Quadrant::BottomLeft {
                    found = true;
                }
            }
            if let Some(&pos) = items.get(&12) {
                debug!("found pos of BottomRight: {pos:?}");
                br = Some(pos);

                if tgt == Quadrant::BottomRight {
                    found = true;
                }
            }

            if car.is_none() || (frame_car.is_some() && found) {
                car = frame_car;
            }

            if w.is_none() || found {
                w = Some(frame.cols() as f64);
            }
            if h.is_none() || found {
                h = Some(frame.rows() as f64);
            }
        }

        let w = w.unwrap();
        let h = h.unwrap();

        match car {
            None => {
                no_car_count += 1;
                warn!("Car marker not found in frame (miss #{no_car_count})");
                if no_car_count > 3 {
                    debug!("Spinning to find marker…");
                    send_cmd(&client, 0.45 * TURN_POLARITY, true).ok();
                }
            }

            Some((pos, heading)) => {
                no_car_count = 0;
                // let tgt_centre = tgt.centre(w, h);
                let Some((tgt_centre, _)) = (match tgt {
                    Quadrant::TopLeft => tl,
                    Quadrant::TopRight => tr,
                    Quadrant::BottomLeft => bl,
                    Quadrant::BottomRight => br,
                }) else {
                    error!("Couldn't find target location");
                    thread::sleep(Duration::from_millis(100));
                    continue;
                };

                let car_quad = Quadrant::from_pos(pos.0, pos.1, w, h);
                let dist = (pos.0 - tgt_centre.0).hypot(pos.1 - tgt_centre.1);

                // if car_quad == tgt {
                info!("distance {dist} from {tgt:?}");
                if dist < 50.0 {
                    info!("✅ In target {:?} — holding position", tgt);
                    send_cmd(&client, 0.0, false).ok();
                } else {
                    let cmd = steer(pos, heading, tgt_centre);
                    info!(
                        "pos=({:.0},{:.0}) hdg={:.2}rad | {:?}→{:?} | speed={:.2} flip={}",
                        pos.0, pos.1, heading, car_quad, tgt, cmd.speed, cmd.flip
                    );
                    if let Err(e) = send_cmd(&client, cmd.speed, cmd.flip) {
                        error!("Drive command failed: {e}");
                    }
                }
            }
        }

        thread::sleep(Duration::from_millis(100));
    }
}

// ─── Helpers ──────────────────────────────────────────────────────────────────

struct Detector {
    dict: opencv::core::Ptr<Dictionary>,
    params: opencv::core::Ptr<DetectorParameters>,
}

fn make_detector() -> Result<Detector> {
    let dict = get_predefined_dictionary(PREDEFINED_DICTIONARY_NAME::DICT_4X4_50)
        .context("ArUco dictionary")?;
    let params = DetectorParameters::create()?;

    Ok(Detector { dict, params })
}

fn fetch_frame(client: &Client, url: &str, auth: &str) -> Result<opencv::core::Mat> {
    let bytes = client
        .get(url)
        .header("Authorization", auth)
        .send()?
        .error_for_status()?
        .bytes()?;

    let buf: Vector<u8> = Vector::from_iter(bytes.iter().copied());
    let img = imgcodecs::imdecode(&buf, imgcodecs::IMREAD_COLOR)?;
    if img.empty() {
        bail!("imdecode returned empty Mat (bad JPEG?)");
    }
    Ok(img)
}

/// Returns (centre_xy, heading_radians) for our car marker, or None if not seen.
fn detect_car(
    detector: &Detector,
    frame: &opencv::core::Mat,
) -> Result<HashMap<i32, ((f64, f64), f64)>> {
    let mut corners: Vector<opencv::core::Mat> = Vector::new();
    let mut ids = opencv::core::Mat::default();
    let mut rejected: Vector<opencv::core::Mat> = Vector::new();

    let mut items = HashMap::new();

    aruco::detect_markers(
        frame,
        &detector.dict,
        &mut corners,
        &mut ids,
        &detector.params,
        &mut rejected,
    )?;

    let n = ids.rows();
    debug!("Detected {n} marker(s) in frame");

    for i in 0..n {
        let id = *ids.at_2d::<i32>(i, 0)?;
        debug!("  marker id={id}");

        // corners[i] is a 1×4 Mat of Point2f (TL, TR, BR, BL order)
        let m = corners.get(i as usize)?;
        let c0 = *m.at_2d::<Point2f>(0, 0)?; // top-left
        let c1 = *m.at_2d::<Point2f>(0, 1)?; // top-right
        let c2 = *m.at_2d::<Point2f>(0, 2)?; // bottom-right
        let c3 = *m.at_2d::<Point2f>(0, 3)?; // bottom-left

        let cx = (c0.x + c1.x + c2.x + c3.x) as f64 / 4.0;
        let cy = (c0.y + c1.y + c2.y + c3.y) as f64 / 4.0;

        // Heading: from centre toward mid-point of the top edge (c0→c1).
        // If the car's physical forward direction differs, adjust TURN_POLARITY
        // or add a heading offset here.
        let fx = (c0.x + c1.x) as f64 / 2.0;
        let fy = (c0.y + c1.y) as f64 / 2.0;
        let heading = (fy - cy).atan2(fx - cx);

        debug!("Car found: centre=({cx:.1},{cy:.1}) heading={heading:.3}rad");
        items.insert(id, ((cx, cy), heading));
    }

    Ok(items)
}

/// Compute a drive command to steer from `pos`/`hdg` toward `target`.
fn steer(pos: (f64, f64), hdg: f64, target: (f64, f64)) -> DriveCmd {
    let dx = target.0 - pos.0;
    let dy = target.1 - pos.1;
    let dist = (dx * dx + dy * dy).sqrt();

    // if dist < ARRIVE_PX {
    //     debug!("steer: close enough ({dist:.0}px < {ARRIVE_PX}px) — stop");
    //     return DriveCmd {
    //         speed: 0.0,
    //         flip: false,
    //     };
    // }

    let desired = dy.atan2(dx);
    let mut err = desired - hdg;
    while err > PI {
        err -= 2.0 * PI;
    }
    while err < -PI {
        err += 2.0 * PI;
    }

    debug!("steer: dist={dist:.0}px desired={desired:.2}rad err={err:.2}rad");

    if err.abs() > ANGLE_OK {
        let spd = TURN_POLARITY * if err > 0.0 { 0.2 } else { -0.2 };
        debug!("steer: turning (speed={spd:.2}, flip=true)");
        DriveCmd {
            speed: spd,
            flip: true,
        }
    } else {
        let spd = (dist / 300.0).clamp(0.45, 0.85) as f32;
        debug!("steer: driving forward (speed={spd:.2}, flip=false)");
        DriveCmd {
            speed: spd,
            flip: false,
        }
    }
}

fn query_oracle(client: &Client) -> Result<Quadrant> {
    if let Some(oracle) = option_env!("ORACLE") {
        return Ok(Quadrant::parse(oracle).unwrap());
    }

    let body = client
        .get(ORACLE_URL)
        .header("Authorization", ORACLE_AUTH)
        .send()?
        .error_for_status()?
        .text()?;

    debug!("Oracle raw response: {body:?}");

    // Handle JSON string, JSON object with "quadrant"/"target" key, or plain text.
    let raw = if let Ok(s) = serde_json::from_str::<String>(&body) {
        s
    } else if let Ok(v) = serde_json::from_str::<serde_json::Value>(&body) {
        v.get("quadrant")
            .or_else(|| v.get("target"))
            .and_then(|x| x.as_str())
            .unwrap_or(body.trim())
            .to_string()
    } else {
        body.trim().to_string()
    };

    Quadrant::parse(&raw).with_context(|| format!("unknown quadrant response: {body:?}"))
}

fn send_cmd(client: &Client, speed: f32, flip: bool) -> Result<()> {
    debug!("send_cmd: speed={speed:.2} flip={flip}");
    client
        .put(CAR_URL)
        .header("Content-Type", "application/json")
        .header("Authorization", CAR_AUTH)
        .json(&DriveCmd { speed, flip })
        .send()?
        .error_for_status()?;
    Ok(())
}
