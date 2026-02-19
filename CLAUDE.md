# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

A Rust hackathon project for the Helsing Rust Nation 2026 challenge. A remote-controlled car navigates a physical arena to reach quadrants specified by an "Oracle" server. Computer vision via OpenCV detects ArUco markers on the car and arena corners to determine position and heading.

## Build & Run

```sh
cargo build
cargo run
```

Run with debug logging:
```sh
RUST_LOG=debug cargo run
```

Override oracle target without hitting the network (useful for testing):
```sh
ORACLE=TopLeft cargo run
```

## Key Configuration Constants (src/main.rs)

All tunable parameters are at the top of the file:

| Constant | Purpose |
|---|---|
| `CAR_MARKER_ID` | ArUco marker ID on the car (currently 9) |
| `ARRIVE_PX` | Pixel distance to target considered "arrived" |
| `ANGLE_OK` | Heading error (rad) threshold: below this, drive forward instead of spin |
| `TURN_POLARITY` | Set to -1.0 or 1.0 to fix if car turns the wrong way |

Network endpoints (`CAR_URL`, `CAM1_URL`, `CAM2_URL`, `ORACLE_URL`) and auth tokens are also defined here.

## Architecture

Single binary (`drive`) with one main control loop in `src/main.rs`:

1. **Oracle poll** (every 2s) — `query_oracle()` GETs the target quadrant. Accepts JSON or plain-text responses, handles `"quadrant"` and `"target"` keys.
2. **Camera frames** — `fetch_frame()` GETs a JPEG from each camera and decodes it with OpenCV.
3. **ArUco detection** — `detect_car()` runs OpenCV's ArUco detector (DICT_4X4_50) on each frame. Returns a `HashMap<marker_id, (centre_xy, heading_rad)>` for all detected markers. Arena corner markers have fixed IDs: 13=TopLeft, 11=TopRight, 14=BottomLeft, 12=BottomRight.
4. **Steering** — `steer()` computes a `DriveCmd {speed, flip}`. If heading error > `ANGLE_OK`, it spins in place (`flip=true`); otherwise drives forward with speed proportional to distance.
5. **Drive command** — `send_cmd()` PUTs JSON `{"speed": f32, "flip": bool}` to the car with a 100ms loop cadence.

When the car marker is not visible for 3+ consecutive frames, the car spins to try to make it visible.

## OpenCV / Build Environment

The crate uses `opencv = "0.98.1"` with `clang-runtime` feature. The `libclang.so` symlink in the repo root is used to satisfy the clang runtime requirement without Nix. If building in a fresh environment, you may need to set `LIBCLANG_PATH`.

The `flake.nix` provides a dev shell with OpenCV, libclang, and pkg-config for Nix users.
