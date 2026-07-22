# TODO List for 3D Game Engine Project

## Project Setup
- [x] Create project directory structure
- [x] Create CMakeLists.txt for build system
- [x] Create .vscode/tasks.json and launch.json for g++ auto run fix

## Core Engine Components
- [x] Implement Vector3D class (src/math/Vector3D.h, Vector3D.cpp)
- [x] Implement Ray class (src/math/Ray.h, Ray.cpp)
- [x] Implement Camera class (src/rendering/Camera.h, Camera.cpp)
- [x] Implement Material class (src/rendering/Material.h, Material.cpp)
- [x] Implement Sphere class (src/objects/Sphere.h, Sphere.cpp)
- [x] Implement Scene class (src/rendering/Scene.h, Scene.cpp)
- [x] Implement RayTracer class (src/rendering/RayTracer.h, RayTracer.cpp)
- [x] Implement PhysicsEngine class (src/physics/PhysicsEngine.h, PhysicsEngine.cpp)
- [x] Implement RigidBody class (src/physics/RigidBody.h, RigidBody.cpp)
- [x] Create main.cpp with game loop

## Documentation and Licensing
- [x] Create README.md
- [x] Create LICENSE (MIT)

## Interactivity Features
- [x] Implement Player class (src/physics/Player.h, Player.cpp) — position/velocity, gravity, jump(), move() all implemented and unit-tested.
- [ ] Wire Player controls into a real input loop — no main_*.cpp reads keyboard/mouse input; UI.cpp/GUI.cpp (SDL) exist but are not built or invoked by any executable. There is currently no playable/interactive build, only single-shot and offline-simulated ray-traced renders to PPM files.
- [x] Add gravity to player (Player::update applies -9.81 m/s^2, verified)
- [x] Add grid support to Material and RayTracer (verified: renders correctly)
- [ ] SDL2 windowing (UI.cpp/GUI.cpp) — intentionally disabled in CMakeLists.txt ("SDL2 linking removed for now to fix build errors"); left disabled, not re-enabled.

## Testing and Finalization
- [x] Build and test the project — GameEngine, Blocks, Parachutes targets all build clean with g++ -std=c++17 -Wall -Wextra -O2 (matches CMakeLists.txt flags); CMake itself unavailable in this sandbox (no apt access), so built directly with g++ using the same target settings CMakeLists.txt specifies.
- [x] Unit-test math/physics core (tests/test_math_physics.cpp): Vector3D ops, Ray-Sphere intersection, RigidBody free-fall integration vs. analytic formula, drag/terminal-velocity behavior.
- [x] Verified all three demo scenes actually render their subjects (not just sky/ground) via pixel-level checks on the output PPMs — this caught and led to fixing a real camera-frustum bug (see below).
- [ ] No display/X server in this sandbox — cannot verify a live interactive window (moot anyway, see above: no such loop exists yet).
