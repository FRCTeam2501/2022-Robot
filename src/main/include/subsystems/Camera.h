#pragma once
#include "frc2/command/SubsystemBase.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include "cscore_oo.h"

namespace Cameras{
    constexpr int with = 300;
    constexpr int height = 200;
    constexpr int fps = 25;
}

class Camera : public frc2::SubsystemBase {
public:
    
     Camera();
     ~Camera();

    void SwitchFeed();

    
    void Periodic();
    void init();
 private:
    cs::UsbCamera *climbCamera, *intakeCamera;
    cs::MjpegServer *streamVideo;

    int currentFeed; 
};


