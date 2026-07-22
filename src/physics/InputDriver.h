#ifndef INPUTDRIVER_H
#define INPUTDRIVER_H

#include <vector>
#include "InputState.h"

// Feeds a pre-recorded sequence of InputState frames to the control loop, one
// per poll(). This is what makes the control loop testable and demo-able
// headlessly: no keyboard, no window, just a script. Past the end of the
// script it returns a neutral (all-zero) state rather than looping or
// throwing, so a demo can keep rendering settle-down frames after the last
// scripted action.
class ScriptedInputDriver {
public:
    explicit ScriptedInputDriver(std::vector<InputState> frames) : frames_(std::move(frames)) {}

    InputState poll() {
        if (cursor_ >= frames_.size()) return InputState{};
        return frames_[cursor_++];
    }

    bool exhausted() const { return cursor_ >= frames_.size(); }

private:
    std::vector<InputState> frames_;
    size_t cursor_ = 0;
};

// A real keyboard/mouse driver. Guarded out entirely unless raylib is linked:
// only the bundled raylib.h header is present in this environment (no
// library, `pkg-config --exists raylib` fails, and there is no sudo/network
// to install one), so this is written to show the intended wiring but is
// never compiled or run here.
#ifdef PHOTON_USE_RAYLIB
#include "raylib.h"

class RaylibInputDriver {
public:
    InputState poll() {
        InputState in;
        if (IsKeyDown(KEY_W)) in.moveForward += 1.0;
        if (IsKeyDown(KEY_S)) in.moveForward -= 1.0;
        if (IsKeyDown(KEY_D)) in.moveRight += 1.0;
        if (IsKeyDown(KEY_A)) in.moveRight -= 1.0;
        in.jump = IsKeyDown(KEY_SPACE);

        Vector2 mouseDelta = GetMouseDelta();
        in.lookYawDelta = mouseDelta.x * 0.003f;
        in.lookPitchDelta = -mouseDelta.y * 0.003f;
        return in;
    }
};
#endif // PHOTON_USE_RAYLIB

#endif // INPUTDRIVER_H
