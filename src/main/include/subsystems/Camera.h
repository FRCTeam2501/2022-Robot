#pragma once
#include "frc2/command/SubsystemBase.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include "cscore_oo.h"
#include <cameraserver/CameraServer.h>

namespace Cameras
{
    //These need to be constexpr to not get a linker error
    constexpr int with = 360;
    constexpr int height = 240;
    constexpr int fps = 15;
}

class Camera : public frc2::SubsystemBase
{
public:
//Constructor and destructor
    Camera();
    ~Camera();

    //The Switch feed function declared void because it is not returning a value
    void SwitchFeed();

private:

    //we make the pointers
    cs::UsbCamera *climbCamera, *intakeCamera;
    cs::MjpegServer *streamVideo;
   

    int currentFeed;
};
