#pragma once
#include "frc2/command/SubsystemBase.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include "cscore_oo.h"

namespace Cameras
{
    constexpr int with = 360;
    constexpr int height = 240;
    constexpr int fps = 15;
}

class Camera : public frc2::SubsystemBase
{
public:
    Camera();
    ~Camera();

    void SwitchFeed();

private:
    cs::UsbCamera *climbCamera, *intakeCamera;
    cs::MjpegServer *streamVideo;

    int currentFeed;
};
