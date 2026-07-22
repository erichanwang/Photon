# Photon

A 3D ray-traced rendering engine in C++, with a rigid-body physics simulation
running on real gravity and air-resistance values. Built from scratch: no
third-party rendering or physics library, just the math.

The renderer does Blinn-Phong shading with cast shadows and recursive
reflections, anti-aliases with jittered supersampling, and accelerates
intersection with a bounding volume hierarchy across a thread pool.

## What's here

- **Ray tracer** (`src/rendering`): `RayTracer` fires camera rays through
  each pixel, intersects the `Scene`, and shades hits with ambient +
  Lambertian diffuse + Blinn-Phong specular per light, casting a shadow ray
  to each. Reflective materials recurse up to `maxDepth` bounces.
  `samplesPerPixel > 1` enables jittered supersampling for anti-aliasing.
- **Dielectrics** (`Material::dielectric`): materials with a `transparency`
  above 0 refract by Snell's law, and the split between the reflected and
  transmitted ray comes from Schlick's approximation of the Fresnel term, so
  glass is a window head-on and a mirror at a glancing angle. Past the
  critical angle there is no transmitted ray at all and the surface reflects
  totally, which is the bright rim along the bottom edge of a glass sphere.
- **Acceleration** (`src/rendering/BVH.h`): a bounding volume hierarchy over
  the scene's bounded objects, built by median split along the longest axis
  of the centroid bounds, stored as a flat node array. Objects of infinite
  extent (an unbounded `Plane`) have no finite box, so `Scene` keeps them on
  a short linear list and tests both.
- **Threading**: rows are handed out through an atomic cursor rather than
  split statically, because a row through the middle of the scene casts far
  more shadow and reflection rays than one through empty sky.
- **Physics** (`src/physics`): `RigidBody` integrates position and velocity
  under gravity and drag using semi-implicit Euler stepping. `PhysicsEngine`
  advances every body and resolves collisions: a ground plane plus
  impulse-based sphere-sphere response with positional correction split by
  inverse mass. `Player` layers movement, jumping, and gravity on top of that.
- **Control loop** (`src/physics/InputState.h`, `InputDriver.h`): `InputState`
  is a plain struct of movement axes, a jump flag, and look deltas, with no
  dependency on any windowing library. `Player::applyInput()` turns one
  `InputState` plus a timestep into a real per-frame control loop: jump, move,
  integrate gravity, land. `ScriptedInputDriver` feeds it a recorded sequence
  of frames headlessly; `RaylibInputDriver` (guarded behind
  `#ifdef PHOTON_USE_RAYLIB`) shows the real-keyboard wiring but is not
  compiled by any target here, since no raylib library is installed. See
  Known limitations for exactly what that does and doesn't prove.
- **Objects** (`src/objects`): `Sphere`, `Block`, `Plane`, `Slope`, and
  `Parachute`, each implementing ray intersection and a bounding box against
  the shared `Object` interface.
- **Math** (`src/math`): `Vector3D`, `Ray`, and `AABB`.
- Four demo scenes: `main.cpp` (shading, shadows, reflections, 4x AA),
  `main_blocks.cpp` and `main_parachutes.cpp` (both animated, with
  collisions), and `main_control_demo.cpp` (drives `Player` through a
  scripted `InputState` sequence — walk, jump, land — rendering one frame per
  input frame, to prove the control loop is real and not dead code).

## Building

```sh
cmake -B build
cmake --build build
```

This produces six targets: `GameEngine`, `Blocks`, `Parachutes`, `Tests`,
`Benchmark`, and `ControlDemo`. Without CMake, each compiles directly, since
none of them depend on anything outside this repo:

```sh
g++ -std=c++17 -O2 -pthread src/main.cpp -o build/GameEngine
```

`-pthread` is required: the renderer's row-sharing loop uses `std::thread`.

SDL2 is referenced in `CMakeLists.txt` but commented out. An earlier attempt
to link it broke the build, so it is disabled rather than left broken.
Nothing in this engine depends on it.

## Testing

`tests/test_math_physics.cpp` is an assert-based self-check, no framework: it
builds as the `Tests` target and exits non-zero if any assertion fails.

```sh
./build/Tests
```

It covers `Vector3D` arithmetic and edge cases (including zero-vector
normalization, which must not divide by zero), ray-sphere intersection (hit,
miss, and behind-the-camera cases), free-fall against the closed-form
semi-implicit-Euler formula, mass-independence of free-fall, drag against the
analytic terminal-velocity bound, and:

- **BVH equals linear scan.** 425 rays against a 61-object scene, asserting
  the accelerated hit record is identical to the exhaustive scan's, field by
  field. The BVH is only an optimization, so the property that matters is
  that it changes nothing; a tree that is fast and subtly wrong is worse than
  the scan it replaced.
- **Ground collision.** A body dropped from 10m settles exactly on the ground
  plane with zero residual velocity, instead of sinking through it.
- **Momentum conservation.** Two elastic spheres collide head-on; total
  momentum along the collision axis is unchanged to 1e-9, and they end up
  separating and non-overlapping.
- **Threaded render equals single-threaded render.** The same scene rendered
  on 1 thread and on 8 must be bit-identical, which is checkable only because
  the supersampling jitter is deterministic. A data race in the row-sharing
  loop shows up here as a mismatched pixel.
- **Shadow rays.** A point behind an occluder is shadowed; a point beside it
  is not.
- **Refraction against Snell's law.** A ray entering glass at 45 degrees must
  leave at `asin(sin(45)/1.5)`, checked to 1e-9 rather than by eyeballing a
  render. The critical angle is checked from both sides: 0.05 rad inside it
  still transmits, 0.05 rad past it must report total internal reflection
  instead of returning a NaN direction from a negative square root. A second
  test renders a white sphere against a red wall twice, opaque and as glass,
  and asserts the opaque pixel is gray while the glass pixel is red-dominant
  - the wall's color can only reach the camera by transmission.
- **Control loop.** A scripted `InputState` sequence drives `Player` through
  `applyInput()`: it falls and lands on spawn without sinking through the
  ground, walking input translates position, jumping leaves the ground, and
  gravity brings it back to rest at `y=0` afterward.

## Benchmarks

```sh
./build/Benchmark [spheres] [repeats]     # defaults: 500 spheres, best of 3
```

Measured on a 16-core machine, 500 spheres plus an unbounded plane, 400x300,
2 samples/pixel, 3 bounces, best of 5:

**Intersection, single-threaded:**

| Structure | Time | Throughput |
|---|---|---|
| Linear scan | 3.345 s | 0.17 Mrays/s |
| BVH (depth 9, 511 nodes) | 0.405 s | 1.40 Mrays/s |
| **Speedup** | **8.3x** | |

**Thread scaling, with the BVH enabled:**

| Threads | Time | Throughput | Speedup |
|---|---|---|---|
| 1 | 0.476 s | 1.19 Mrays/s | 1.0x |
| 2 | 0.261 s | 2.17 Mrays/s | 1.8x |
| 4 | 0.169 s | 3.35 Mrays/s | 2.8x |
| 8 | 0.091 s | 6.25 Mrays/s | 5.3x |
| 16 | 0.069 s | 8.24 Mrays/s | 6.9x |

The benchmark verifies that the BVH and the linear scan produce identical
images, and that every thread count produces the image the single-threaded
run did, before reporting any timing. A speedup can therefore never come from
one configuration quietly doing less work.

Scaling falls short of linear mainly because this scene renders in under a
second, so thread startup and the tail of the last few rows are a real
fraction of the total. An earlier version also incremented a shared atomic
ray counter on every ray; that single contended cache line cost roughly half
the achievable scaling, and the counter is now thread-local and folded in
once per worker.

## Known limitations

The control loop itself is real and tested, but nothing here reads a real
keyboard, because raylib is not available in this environment: only the
bundled `raylib.h` header is present, no library is installed, and
`pkg-config --exists raylib` fails (confirmed; `apt-get install libraylib-dev`
also fails here for lack of sudo). To keep `Player::move()`/`jump()` genuinely
reachable rather than dead code without that dependency, the control loop was
split from any input source:

- `InputState` (`src/physics/InputState.h`) is a plain struct — movement
  axes, jump flag, look deltas — with no windowing dependency.
- `Player::applyInput()` (`src/physics/Player.h`) is the actual control loop:
  jump, move, integrate gravity, land. This is exercised end to end, headlessly:
  `tests/test_math_physics.cpp`'s `testControlLoop` asserts landing, walking,
  jumping, and re-landing against real trajectories, and `ControlDemo`
  (`src/main_control_demo.cpp`) drives the same loop through 30 scripted
  frames and ray-traces each one, so the player's motion is visible in the
  rendered output, not just in printed numbers.
- `ScriptedInputDriver` (`src/physics/InputDriver.h`) is what feeds both of
  those a recorded sequence of frames.
- `RaylibInputDriver`, in the same file, is written to show the intended
  wiring to real keyboard/mouse input, guarded behind
  `#ifdef PHOTON_USE_RAYLIB` so it compiles into nothing by default. It has
  never been compiled or run in this environment — there is no way to verify
  it without a raylib library to link, and no interactive window can be
  confirmed here (no display/X server either). Getting from here to a
  playable build is linking raylib and passing `-DPHOTON_USE_RAYLIB`; no
  other code changes are expected to be needed.

Collision treats every body as a sphere, so blocks resolve against their
bounding sphere rather than their faces and will not come to rest on a corner
realistically. The broad phase is an O(n^2) pair loop, which is fine for the
tens of bodies these demos use.

The BVH splits at the median rather than by a surface-area heuristic, and is
rebuilt from scratch each frame in the animated demos rather than refitted.
`UI.cpp` and `GUI.cpp` are SDL-based and are not compiled by any target while
SDL2 stays disabled.

Refraction is uniform: there is no wavelength-dependent index, so the renderer
produces no chromatic dispersion, and no Beer-Lambert absorption, so thick
glass tints exactly as much as thin glass.

## License

MIT. See [LICENSE](LICENSE).
