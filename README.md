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
  the scene's bounded objects, stored as a flat node array. `BVH::Heuristic`
  selects how it splits: `SAH` (the default) bins primitives by centroid on
  each axis and sweeps for the split that minimizes expected traversal cost,
  falling back to a median split if no split beats leaving the range as a
  leaf; `Median` always splits at the middle of the longest axis of the
  centroid bounds. Objects of infinite extent (an unbounded `Plane`) have no
  finite box, so `Scene` keeps them on a short linear list and tests both.
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
  field, for both the median-split and SAH builds independently. The BVH is
  only an optimization, so the property that matters is that it changes
  nothing; a tree that is fast and subtly wrong is worse than the scan it
  replaced.
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
| Linear scan | 2.553 s | 0.22 Mrays/s |
| BVH, SAH (depth 11, 583 nodes) | 0.319 s | 1.77 Mrays/s |
| **Speedup** | **8.0x** | |

**Median split vs binned SAH, single-threaded:**

| Heuristic | Build time | Depth | Nodes | Render time | Throughput |
|---|---|---|---|---|---|
| Median split | 0.0004 s | 9 | 511 | 0.339 s | 1.67 Mrays/s |
| Binned SAH | 0.0008 s | 11 | 583 | 0.330 s | 1.71 Mrays/s |

On this scene SAH wins, but only barely: about 3% faster traversal for
roughly 2x the build time, and both are sub-millisecond to build regardless.
The scene is 500 spheres scattered fairly evenly around a ring, which is
close to the case median split already handles well; SAH's advantage grows
on scenes with uneven or clustered object density, where a fixed midpoint
split leaves lopsided subtrees and SAH's cost search does not. The benchmark
verifies the two heuristics render bit-identical images before reporting
either number, so this is a real (if modest) result, not a rounding artifact.

**Thread scaling, with the BVH enabled:**

| Threads | Time | Throughput | Speedup |
|---|---|---|---|
| 1 | 0.320 s | 1.76 Mrays/s | 1.0x |
| 2 | 0.196 s | 2.89 Mrays/s | 1.6x |
| 4 | 0.120 s | 4.71 Mrays/s | 2.7x |
| 8 | 0.080 s | 7.03 Mrays/s | 4.0x |
| 16 | 0.057 s | 9.89 Mrays/s | 5.6x |

The benchmark verifies that the BVH and the linear scan produce identical
images, that the median-split and SAH builds produce identical images to
each other, and that every thread count produces the image the
single-threaded run did, before reporting any timing. A speedup can
therefore never come from one configuration quietly doing less work.

Scaling falls short of linear mainly because this scene renders in under a
second, so thread startup and the tail of the last few rows are a real
fraction of the total. An earlier version also incremented a shared atomic
ray counter on every ray; that single contended cache line cost roughly half
the achievable scaling, and the counter is now thread-local and folded in
once per worker.

### Re-verified, different machine

The numbers above were never re-measured until now. Re-run today on a
16-thread laptop (Intel Core i7-1360P, 12 cores/16 threads, hybrid P+E, not
the machine the original table was measured on) with `./build/Benchmark 500
8`:

| Metric | Original table | This machine |
|---|---|---|
| BVH vs linear scan, 1 thread | 8.0x | 6.3-8.2x (noisy, see below) |
| Peak throughput, 16 threads | 9.89 Mrays/s | 5.2-6.9 Mrays/s |
| Thread scaling, 16 threads | 5.6x | 5.4-8.7x (noisy, see below) |

Two honest caveats on these numbers, since the point of this exercise was to
stop taking benchmark output on faith:

- **This machine is a laptop, not the original 16-core box**, and its
  `hardware_concurrency() == 16` is 12 physical cores plus hyperthreading,
  not 16 physical cores -- a weaker chip for sustained parallel work than the
  number alone suggests. The peak-throughput shortfall against the original
  9.89 Mrays/s is consistent with that, not a regression.
- **Repeated back-to-back runs measurably throttle this laptop.** The same
  `500 8` invocation returned single-thread throughput anywhere from 0.44 to
  0.91 Mrays/s depending on how much benchmarking had already run in the
  session (thermal ramp, plus another process on this shared machine during
  part of the session). Best-of-N within one run cancels scheduling noise;
  it does nothing for a CPU package that is genuinely slower ten minutes
  into a benchmarking session than it was at the start. The range above
  spans several separate invocations, not one; treat the low end as the
  believable sustained number and the high end as a cool-CPU best case.

Bottom line: the **9.9M rays/sec** peak-throughput figure was not
reproduced on this hardware -- best observed here is in the 5-7 Mrays/s
range. The **8.0x BVH-vs-linear-scan** and **5.6x thread-pool** figures both
landed within or above their original range across repeated runs, so those
two hold up; throughput is the one that's genuinely hardware-bound.

### Optimizations added and measured this pass

All three changes below were checked against `tests/test_math_physics.cpp`'s
`testBVHMatchesLinearScan` and `testThreadedRenderMatchesSingleThreaded`
after each edit -- same images, not just plausible ones -- before being kept.

- **Hoist the BVH slab test's `1/direction` out of the traversal loop**
  (`src/math/AABB.h`, `src/rendering/BVH.h`). `AABB::hit()` was computing
  three divisions on every node it visited, but a ray's direction does not
  change between node visits within one `BVH::intersect()` call. `AABB` now
  has a second `hit()` overload that takes a precomputed `1/direction`, and
  `BVH::intersect()` computes it once per ray instead of once per node. This
  is strictly less arithmetic per node for any tree depth greater than one --
  not something that needs a benchmark to justify -- but isolating its wall-clock
  effect from the thermal noise described above was not possible this pass.
- **Stop re-normalizing `ray.direction`** (`src/rendering/RayTracer.h`).
  `Ray`'s constructor already normalizes `direction` once
  (`src/math/Ray.h`), so the four call sites in `background()`, `trace()`
  (both the reflective and transparent branches), and `shade()` that called
  `.normalize()` on it again were recomputing a square root on an
  already-unit vector, on every primary ray, every reflection bounce, and
  every shading point. Removed; `rec.normal.normalize()` is left alone since
  `Plane`/`Slope` take a caller-supplied normal that is not provably unit.
  Same reasoning as the slab-test hoist: guaranteed less work, effect not
  cleanly separable from machine noise this pass.
- **SoA sphere-leaf traversal** (`src/objects/Sphere.h`, `src/rendering/BVH.h`,
  pre-existing work-in-progress found on this branch, finished and verified
  here): BVH leaves store spheres' center/radius in flat parallel arrays
  alongside the existing `Object*` list, so a leaf test reads four contiguous
  doubles instead of dereferencing a heap-allocated `Sphere` through a vtable
  call, via a new `Sphere::intersectAt()` that both the member `intersect()`
  and the BVH leaf path call. Verified correct (bit-identical images,
  `testBVHMatchesLinearScan` and the full benchmark's image-match gates all
  pass). Its wall-clock effect was inconclusive on this machine: paired
  before/after runs of the same scene came back anywhere from a slight win
  to a slight loss, smaller than the thermal/scheduling noise floor
  described above. Kept because it does less work per candidate on paper and
  breaks nothing; not claimed as a proven speedup.

### GPU port status

No CUDA toolkit is installed on this machine (`nvcc` not found, no
`/usr/local/cuda*`), and no Vulkan SDK/shader compiler either (`glslc`,
`glslangValidator` both absent) -- only the Vulkan loader and Mesa's
software/ICD drivers are present, which is not enough to compile a compute
shader. So "CUDA/Vulkan port underway" was, before this pass, aspirational:
there was no GPU code anywhere in the repository.

`src/gpu/raytrace_gpu.cu` is a real starting point, not a stub: a CUDA kernel
that traces primary rays against a flat sphere array (brute-force per-sphere
loop, one thread per pixel), shades with ambient + Lambertian diffuse +
Blinn-Phong specular under one directional light with one shadow ray, and a
host `main()` that builds the identical scene twice -- once through the real
CPU `Scene`/`RayTracer`/`Camera` classes, once flattened into device arrays
-- races both, and diffs the two images before printing a rays/sec number
for either, refusing to report a GPU number that doesn't match the CPU
reference (same correctness-before-speed pattern as `bench/benchmark.cpp`
and `bench/broadphase_bench.cpp`). `CMakeLists.txt` picks it up as an
optional `GpuRayTrace` target via `check_language(CUDA)`, so every existing
CPU target still builds unmodified on a machine without CUDA, which is every
machine this repository has been built on so far, including this one.

**This has not been compiled or run.** There is no CUDA toolchain here to
compile it with, so its numbers do not exist yet and are not claimed. What
it does not yet cover, honestly: BVH traversal on the device (this kernel
does the same brute-force scan as the CPU's pre-BVH baseline, which is the
correct first correctness target, not the fast path), the ground plane,
reflection/refraction, multiple lights, and soft shadows/depth of field --
all real gaps against the CPU renderer, not hidden ones.

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

The BVH is rebuilt from scratch each frame in the animated demos rather than
refitted, regardless of which split heuristic is selected.
`UI.cpp` and `GUI.cpp` are SDL-based and are not compiled by any target while
SDL2 stays disabled.

Refraction is uniform: there is no wavelength-dependent index, so the renderer
produces no chromatic dispersion, and no Beer-Lambert absorption, so thick
glass tints exactly as much as thin glass.

## License

MIT. See [LICENSE](LICENSE).
