# basic_control

Minimal point-to-point control for a **single Kinova Gen3 7-DoF** arm, using
only the Kinova **Kortex high-level Base API**. It does **not** use the HumanSL
planning stack (GTSAM / GPMP2 / Pinocchio / Vicon) — it links only the bundled
`../../third_party/kortex_api`, so it builds in seconds with no extra dependencies.

Motions run under Kinova's supervised **high-level servoing**, so joint/Cartesian
limits and self-collision protection stay active, and speeds are capped.

## Build

```bash
cd Christian_control/basic_control
mkdir -p build && cd build
cmake .. && make
```

## Use

> Run from `basic_control/build/`. Default arm IP is `192.168.1.10` (override with `--ip`).

```bash
./basic_control state                          # read-only: print joint angles + tool pose
./basic_control home                           # run the arm's stored "Home" action
./basic_control joints 0 15 180 230 0 55 90    # go to a joint config (degrees)
./basic_control pose 0.45 0.0 0.45 90 0 90     # move tool to a Cartesian pose (m, deg)
./basic_control gripper 0.0                     # open gripper (1.0 = closed)
```

Options: `--ip ADDR`, `--speed DEG/S` (joint moves, default 20), `--lin M/S`
(Cartesian moves, default 0.10).

### Typical "move from A to B"

```bash
./basic_control state                          # 1. see where it is now
./basic_control home                           # 2. go to a known safe pose
./basic_control joints 0 15 180 230 0 55 90    # 3. point A
./basic_control joints 0 -20 180 230 0 -40 90  # 4. point B
```

## Safety

- **First moves: keep the workspace clear and the e-stop in hand.**
- Try `state` first (it never moves the arm) to confirm the connection.
- Start with a low `--speed` (e.g. `--speed 10`) until you trust a target.
- For `pose`, the target must be reachable; the robot will abort (not force) an
  unreachable or limit-violating goal and the tool prints `motion ABORTED`.
- Joint angles are absolute, in degrees, in the arm's base frame — read them
  off `state` first to understand the current configuration.
