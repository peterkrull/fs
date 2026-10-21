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
| `--launch-speed M/S` | airspeed when launched, default that of the aircraft, or 14 |
| `--weather PATH` | weather definition, see [Weather](#weather) |
| `--wind FROM,SPEED` | wind above the boundary layer, from a heading in degrees, in m/s |
| `--turbulence SEVERITY` | `light`, `moderate`, `severe`, or the intensity in m/s |
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
| F1 | force arrows: lift (green, red when stalled) and drag (magenta) per meter of span, bodies (blue) per meter of length, thrust (yellow), and the total lift at the center of lift (cyan) |
| F2 | the aerodynamic model: strips, control surfaces as deflected, bound vortices, body cells, rotor discs, center of gravity |
| F3 | hide the 3D model |
| F4 | the wind blowing at the aircraft, and the thermals nearby as rings at its height |
| F11 | fullscreen |

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
- **Sweep** is accounted for with simple sweep theory: only the flow across the swept quarter
  chord line makes lift. This lowers the lift slope of swept surfaces, and makes a swept wing
  roll away from a sideslip, like dihedral does.
- The strips interact through a **nonlinear lifting line**: each carries a horseshoe vortex,
  whose trailing vortices induce downwash on all strips. This gives the loss of lift towards the
  tips, induced drag, the downwash of the wing on the tail, and with that the center of lift as it
  moves with the flow and the controls. No flow is simulated: it is a small system of equations,
  solved iteratively from the previous step.
- The **wake** leaves the trailing edges along the flow, so the downwash at the tail follows the
  angle of attack and the sideslip. The tail sees the wake the wing shed a moment ago, as it
  takes that long to get there, which adds to the damping in pitch.
- Beyond the **stall**, the vortex strengths are smoothed along the span where the lift falls
  with the angle of attack, so the stall spreads smoothly, from where it starts, such as the
  root of a rectangular wing.
- **Control surfaces** are flaps that change the effective camber of their strips, after thin
  airfoil theory, and lose effectiveness at large deflections. They raise the maximum lift by
  less than the lift they add, so that landing flaps lower the stall speed.
- **Bodies** have crossflow and axial drag along their length, and the lift of slender body
  theory, which turns them across the flow (the Munk moment) and makes a fuselage destabilizing.
- **Rotors** have thrust and power coefficients by the advance ratio, either from a simple model
  or a table of measurements. They are driven by a brushless motor, whose battery sags under
  load, or by a piston engine, whose power falls with the density of the air. The rotor has an
  inertia of its own, which gives the throttle a realistic delay, and the airframe the reaction
  torque and gyroscopic moments of its rotors. With the engine off, a propeller windmills. Surfaces and bodies behind a rotor are in its **slipstream**, after momentum theory, so
  a tail behind a propeller keeps some authority even without airspeed.
- **Actuators** follow their commands with a first order response and a rate limit.
- **Wind** varies over the aircraft: it is sampled at both ends of each axis of the aircraft, so
  that a gust hits one wing harder than the other, and each strip, body cell and rotor sees the
  wind where it is. See [Weather](#weather).

The sensors report what a flight controller would measure, without noise: angular rates and
specific force in the body frame, position, velocity, attitude, airspeed and battery state.

## Weather

The weather is defined in RON, see [`assets/weather`](assets/weather) for examples and
[`src/sim/weather`](src/sim/weather) for the documented schema, and reloaded whenever it is
saved. `()` is calm air in the standard atmosphere, and `--wind` and `--turbulence` override
the definition.

```bash
cargo run --release -- --weather assets/weather/breezy.ron    # wind, gusts, light turbulence
cargo run --release -- --weather assets/weather/soaring.ron   # thermals and ridge lift
cargo run --release -- --weather assets/weather/front.ron     # a low moving in
```

The weather is a function of where and when it is sampled, from a seed, without state of its
own: it is the same however often it is sampled, and the same seed gives the same weather.
What drifts with the air (turbulence, gusts and thermals) drifts with the wind aloft at the spawn
point.

- The **wind aloft** is the `wind` plus the geostrophic wind around **pressure systems**: highs
  and lows, which may move. Below the top of the **boundary layer**, the wind slows down
  logarithmically towards the ground, and turns towards low pressure.
- **Gusts** add to the wind aloft for a few seconds, rising and falling as one minus a cosine.
- **Turbulence** is a sum of random waves with the von Kármán spectrum, with the intensities and
  scales of MIL-HDBK-1797, which shrink towards the ground.
- **Thermals** rise from the ground to the top of the convective layer, and are ringed by sinking
  air, after the model of Allen (2006). They come and go, and drift with the wind.
- **Ridge lift** follows the slope of the terrain below the wind. Only the slope right below the
  aircraft is known, so the lift fades with the height above it.
- The **air** is the standard atmosphere from the pressure and temperature at sea level, which the
  pressure systems change. Its density sets the lift, drag and thrust.

ArduPilot is told the wind, but works out the air pressure from the altitude itself, so it does
not see the pressure systems in its barometer.

## Aircraft

| | |
|---|---|
| [`uav.ron`](assets/aircraft/uav.ron) | a 2 m UAV with a top-mounted wing, a conventional tail and a pusher propeller above the wing |
| [`flying_wing.ron`](assets/aircraft/flying_wing.ron) | a 1.5 m flying wing with elevons, winglets with rudders, and a pusher propeller at the back |
| [`cessna172.ron`](assets/aircraft/cessna172.ron) | a Cessna 172P at its maximum weight, compared with the real one, see below |
| [`sbach35.ron`](assets/aircraft/sbach35.ron) | a 35% scale Sbach 342 in the style of a Red Bull racer: 2.6 m span, 12 kg, near-full-span ailerons and a 120 cc flat twin |

```bash
cargo run --release -- --aircraft assets/aircraft/flying_wing.ron
```

The two small aircraft are stable without a pilot: `cargo test` checks for each that its center of
gravity is ahead of its neutral point, that it is trimmed for a sensible speed with the controls
centered, that it glides hands off and holds a bank without spiraling, and that each stick turns
it the right way. The flying wing is stable in pitch without a tail through its sweep and
washout, which make the tips act as a tail, and its reflexed airfoil.

Each comes with ArduPlane parameters, such as the elevon outputs of the flying wing, in the
`.parm` file next to it.

The Sbach is a large-scale aerobatic model rather than a slow UAV: it launches at 27 m/s, trims
hands off at about 24 m/s, and rolls at around 450°/s at cruise. It has big, fast controls for
snaps and quick direction changes, and its own parameters in `sbach35.parm`.
[`tests/sbach35.rs`](tests/sbach35.rs) checks its static margin, glide, control response, roll
rate and climb, and prints them with `cargo test --release --test sbach35 -- --nocapture`.

### The Cessna 172P

The Cessna is built from published data: the dimensions, weights, engine and propeller of its
Pilot's Operating Handbook, the tail and control surface areas of Jane's, the moments of
inertia and propeller coefficients of the JSBSim model, and the airfoil sections of NACA.
Where no data was to be had, three inputs are calibrated, each against a single figure of the
handbook: the drag of the engine cooling and what sticks out (one cruise speed), the propeller
at low speed (the static rpm and the climb at sea level), and the windmilling propeller (the
glide ratio).

[`tests/cessna172.rs`](tests/cessna172.rs) flies it like a test pilot would and compares it with
the handbook and with flight tests, and prints the comparison with `cargo test --release --test
cessna172 -- --nocapture`:

- **Cruise**: within 1 kt of the cruise performance table at all altitudes and engine speeds,
  with the engine power within 5 % of the rated power. High up, full throttle gives a little
  less rpm than the handbook, such as 2570 instead of 2650 at 8000 ft, and so up to 5 kt less.
- **Climb**: within 10 % up to 6000 ft, but declining more slowly with altitude than the
  handbook, by 90 ft/min at 12000 ft, so that the service ceiling is 15500 ft instead of 13000.
  The best climb speed is 79 KIAS, the handbook gives 76.
- **Stall**: 52 KCAS with the flaps up, the handbook 51 to 52, and 48.5 with the flaps at 30°,
  the handbook 46. The slotted flaps of the Cessna keep more of their lift at the stall than
  the plain flaps of the simulation.
- **Glide**: the best glide speed is 65 KIAS, as in the handbook.
- **Maximum speed**: 122 kt at sea level, the handbook 123.
- **Phugoid**: a period of 29 s, damped by about 0.1, where flight tests found 33 s, and 35 to
  40 s, heavily damped, with the elevator free to float rather than held.
- **Lateral**: rolls into a rudder only turn, keeps the slip ball centered in an aileron only
  turn, and slowly spirals into a dive after a nudge of the rudder, as the AAIB found in its
  flight trial (bulletin 12/2020), in 18 s rather than 10 to 15.

```bash
cargo run --release -- --aircraft assets/aircraft/cessna172.ron
```

## Aircraft definitions

Aircraft are defined in [RON](https://github.com/ron-rs/ron), see
[`assets/aircraft/uav.ron`](assets/aircraft/uav.ron) and the documented schema in
[`src/sim/config.rs`](src/sim/config.rs). Positions are in a forward-right-down frame with any
origin, angles in degrees.

The definition is reloaded whenever it is saved, also by `sim-headless`. The aircraft carries on
from where it is, held or in flight, with its controls and motors as they were. A definition
with errors is reported in the flight panel, and the aircraft flies on as it was until it is
fixed.

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
- `rotors`: propellers with their coefficients, an electric `motor` or a piston `engine`, spin
  direction and throttle actuator. Multicopters are rotors pointing up.
- `launch_speed`: the airspeed to launch at, unless given on the command line.
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
