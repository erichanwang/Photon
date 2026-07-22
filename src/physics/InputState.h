#ifndef INPUTSTATE_H
#define INPUTSTATE_H

// Decouples the control loop from any windowing library. Whatever reads a
// real keyboard/mouse (raylib, or anything else) only has to fill one of
// these per frame; Player and the control loop never see the input source.
struct InputState {
    // Movement axes in camera-relative space, each in [-1, 1].
    double moveForward = 0.0; // +1 = forward
    double moveRight = 0.0;   // +1 = right
    bool jump = false;
    // Mouse-look deltas, radians for this frame.
    float lookYawDelta = 0.0f;
    float lookPitchDelta = 0.0f;
};

#endif // INPUTSTATE_H
