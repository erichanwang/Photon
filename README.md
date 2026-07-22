# Photon

A 3D ray-traced rendering engine in C++, with a rigid-body physics
simulation running on real gravity and air-resistance values. Built from
scratch: no third-party rendering or physics library, just the math and
raylib for the window.

## What's here

- **Ray tracer** (`src/rendering`): a `Scene` of objects lit by a
  `RayTracer` that fires camera rays through each pixel, checks
  intersections, and shades hits with `Material` properties. `Camera`
  builds the view frustum from a position, direction, and field of view.
- **Physics** (`src/physics`): `RigidBody` integrates position and
  velocity under gravity and drag using semi-implicit Euler stepping.
  `PhysicsEngine` advances every body in a scene each frame. `Player`
  layers movement and jumping on top of a rigid body.
- **Objects** (`src/objects`): `Sphere`, `Block`, `Plane`, `Slope`, and
  `Parachute`, each implementing ray intersection against the shared
  `Object` interface.
- **Math** (`src/math`): `Vector3D` and `Ray`, the primitives everything
  else is built on.
- Three entry points, each a different demo scene: `main.cpp`,
  `main_blocks.cpp`, `main_parachutes.cpp`.

## Building

```sh
cmake -B build
cmake --build build
```

This produces four targets: `GameEngine`, `Blocks`, `Parachutes`, and
`Tests`. Without CMake, each target also compiles directly, since none
of them depend on anything outside this repo besides the bundled
`raylib.h`:

```sh
g++ -std=c++17 -O2 src/main.cpp -o build/GameEngine
```

SDL2 is referenced in `CMakeLists.txt` but commented out. An earlier
attempt to link it broke the build, so it's disabled rather than left in
a broken state. Nothing in this engine currently depends on it.

## Testing

`tests/test_math_physics.cpp` is a small assert-based self-check, no
framework: it builds as the `Tests` target and exits non-zero if any
assertion fails.

```sh
./build/Tests
```

It covers `Vector3D` arithmetic and edge cases (including
zero-vector normalization, which must not divide by zero), ray-sphere
intersection (hit, miss, and behind-the-camera cases), free-fall against
the closed-form semi-implicit-Euler formula, mass-independence of
free-fall (heavier objects fall at the same rate, as they should), and
drag behavior against the analytic terminal-velocity bound.

## Known limitations

There's no collision system. Objects in the `Blocks` and `Parachutes`
demos fall through the ground once physics has run long enough; this was
never implemented, not a regression. None of the three entry points read
keyboard or mouse input, so despite `Player::move()` and `Player::jump()`
existing and being unit-tested, there's currently no interactive control
loop wiring them to real input. `UI.cpp` and `GUI.cpp` are SDL-based and
aren't compiled by any target while SDL2 stays disabled.

## License

MIT. See [LICENSE](LICENSE).
