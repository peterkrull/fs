# bevy-worldmap

A Google Earth-lite in [Bevy](https://bevyengine.org): satellite or OpenStreetMap imagery draped
over real elevation on a spherical earth, streamed from the web, with an atmosphere. On top of it
runs a flight simulator, which can fly in lockstep with ArduPilot.

```bash
cargo run --release                 # fly the UAV yourself
cargo run --release -- --sitl       # let ArduPilot fly it
cargo run --release -- --no-aircraft
```

| Option | |
|---|---|
| `--aircraft PATH` | aircraft definition, default `assets/aircraft/uav.ron` |
| `--spawn LAT,LON,ALT,HDG` | where the aircraft starts: degrees, meters above sea level, degrees |
| `--launch-speed M/S` | airspeed when launched, default 14 |
| `--wind FROM,SPEED` | constant wind from a heading in degrees, in m/s |
| `--sitl [PORT]` | lockstep with ArduPilot SITL, default port 9002 |
| `--radio ORDER` | use game controllers as RC radios with this channel order, see below |

## Controls

The aircraft starts held in place in the air. **L** launches it at its launch speed, **R** puts it
back. Touching the terrain stops it, until reset.

| Key | |
|---|---|
| Arrows | roll and pitch |
| A / D | yaw |
| W / S | throttle |
| Tab | switch between the chase camera and the free camera |
| Mouse | orbit the chase camera (click to capture, Esc to release, or hold the right button) |
| Scroll | zoom the chase camera |
| F1 | force arrows: lift (green, red when stalled), drag (magenta), bodies (blue), thrust (yellow), and the total lift at the center of lift (cyan) |
| F2 | the aerodynamic model: strips, control surfaces as deflected, bound vortices, body cells, rotor discs, center of gravity |
| F3 | hide the 3D model |

A gamepad flies mode 2: throttle and yaw on the left stick, roll and pitch on the right.

### RC radios

EdgeTX and OpenTX radios connected over USB in **USB Joystick** mode are recognized by their USB
ID (`1209:4F54`), and set the controls directly, with all their resolution and no deadzone. Their
channels are expected in the EdgeTX default order AETR (aileron, elevator, throttle, rudder). For
another order, or another radio that shows up as a game controller, pass it with
`--radio TAER`; a lowercase letter reverses that channel, such as `--radio AeTR`. The flight panel
shows the channels as received, to check the order and directions: aileron right, elevator
forward, throttle up and rudder right should all be positive.

Use the Classic joystick mode of EdgeTX, which reports channels 1 to 8 as axes. Only channels 1 to
6 come through on Linux, as the last two arrive as a slider and a dial, which the game controller
library does not map.

## Flight model

The simulation runs at 1 kHz on a thread of its own, in `f64` earth-centered coordinates, and is
independent of rendering (`src/sim`).

- **Lifting surfaces** are divided into spanwise strips, each with the lift, drag and pitching
  moment of its airfoil at its local angle of attack, which includes the rotation of the aircraft
  and the wind. Beyond the stall, the coefficients blend into those of a flat plate, so any
  attitude is handled, including flying backwards.
- The strips interact through a **nonlinear lifting line**: each carries a horseshoe vortex,
  whose trailing vortices induce downwash on all strips. This gives the loss of lift towards the
  tips, induced drag, the downwash of the wing on the tail, and with that the center of lift as it
  moves with the flow and the controls. No flow is simulated: it is a small system of equations,
  solved iteratively from the previous step.
- **Control surfaces** are flaps that change the effective camber of their strips, after thin
  airfoil theory, and lose effectiveness at large deflections.
- **Bodies** have crossflow and axial drag along their length.
- **Rotors** have thrust and torque coefficients that fall off with the advance ratio, driven by
  a brushless motor model with its own inertia. This gives the throttle a realistic delay, and
  the airframe the reaction torque and gyroscopic moments of its rotors. The battery sags under
  load.
- **Actuators** follow their commands with a first order response and a rate limit.
- **Wind** is anything implementing `WindField`, which may vary in space and time. For now there
  is a constant wind.

The sensors report what a flight controller would measure, without noise: angular rates and
specific force in the body frame, position, velocity, attitude, airspeed and battery state.

## Aircraft

| | |
|---|---|
| [`uav.ron`](assets/aircraft/uav.ron) | a 2 m UAV with a top-mounted wing, a conventional tail and a pusher propeller above the wing |
| [`flying_wing.ron`](assets/aircraft/flying_wing.ron) | a 1.5 m flying wing with elevons, winglets with rudders, and a pusher propeller at the back |

```bash
cargo run --release -- --aircraft assets/aircraft/flying_wing.ron
```

Both are stable without a pilot: `cargo test` checks for every aircraft that its center of
gravity is ahead of its neutral point, that it is trimmed for a sensible speed with the controls
centered, that it glides hands off and holds a bank without spiraling, and that each stick turns
it the right way. The flying wing is stable in pitch without a tail through its sweep and
washout, which make the tips act as a tail, and its reflexed airfoil.

Each comes with ArduPlane parameters, such as the elevon outputs of the flying wing, in the
`.parm` file next to it.

## Aircraft definitions

Aircraft are defined in [RON](https://github.com/ron-rs/ron), see
[`assets/aircraft/uav.ron`](assets/aircraft/uav.ron) and the documented schema in
[`src/sim/config.rs`](src/sim/config.rs). Positions are in a forward-right-down frame with any
origin, angles in degrees.

- `mass`, `center_of_gravity` and `inertia` about it.
- `airfoils`: by name, either `Parametric` (lift slope, zero lift angle, drag polar, pitching
  moment, stall angles) or a `Table` of coefficients by angle of attack, such as from XFoil.
- `surfaces`: wings, tails and fins, each from a root leading edge outwards through `sections`
  with their own span, tip chord, sweep, dihedral, twist, number of cells, airfoil and
  `controls`. A section may set its own `incidence`, such as a winglet at the tip of a twisted
  wing. Surfaces can be mirrored, with their controls deflecting symmetrically (elevators),
  antisymmetrically (ailerons) or by another actuator. A fin is a section with a dihedral of 90°.
- `bodies`: fuselages and pods, as a radius profile along their length.
- `drag`: other drag, such as landing gear or multicopter arms.
- `rotors`: propellers with their motor, spin direction and throttle actuator. Multicopters are
  rotors pointing up.
- `actuators`: servos and speed controllers, with their ArduPilot servo channel, pulse widths,
  dynamics, and how the pilot's sticks drive them (including mixes, such as for elevons).
- `visual`: `Generated` from the geometry above, or a `Gltf` model.

## ArduPilot

In lockstep mode, the simulator talks to ArduPilot SITL through its
[JSON interface](https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON):
ArduPilot sends its servo outputs over UDP, and the simulator steps by its frame time and replies
with the sensor data, including the position as latitude, longitude and altitude. The
simulation thereby runs exactly as fast as ArduPilot, and the rendering never holds it up.

```bash
cargo run --release -- --sitl

# In the ArduPilot repository. The simulator prints this command at startup, with the
# location it spawns at and the parameters of the aircraft.
build/sitl/bin/arduplane --model JSON:127.0.0.1 --serial0 tcp:0 --home 46.6863,7.8632,700,70 \
    --defaults Tools/autotest/models/plane.parm,/path/to/bevy-worldmap/assets/aircraft/uav.parm
```

**`--serial0 tcp:0` matters**: without it, ArduPlane prints `Waiting for connection ....` and
does not start, nor talk to the simulator, until a ground station connects to TCP port 5760.
With it, ArduPlane starts right away, and a ground station can still connect to that port at any
time, such as `mavproxy.py --master tcp:127.0.0.1:5760` or QGroundControl with a TCP link.
`sim_vehicle.py`, which starts MAVProxy itself, should work too, but has not been tested here.

For the flying wing, run the simulator with `--aircraft assets/aircraft/flying_wing.ron`, and
use `flying_wing.parm` in place of `uav.parm`.

Arm, for example in LOITER, and press **L** to launch.
[`scripts/sitl_loiter.py`](scripts/sitl_loiter.py) connects as a ground station, arms, and logs
the flight.

To run several simulations side by side, give each ArduPlane its own instance with `-I N`, which
moves its ports up by `10 * N`, and the simulator the matching port, such as `--sitl 9012` for
`-I 1`.

`sim-headless` runs the same simulation without graphics, over flat ground, for automated tests:

```bash
cargo run --release --bin sim-headless -- --sitl --launch-after 50 --duration 180
```

## Tests

```bash
cargo test
```

Among others, these check the lifting line against theory, and that the UAV glides stably hands
off, holds a bank without spiraling, and responds to each control in the right direction.
