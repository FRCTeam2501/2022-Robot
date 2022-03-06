#pragma once
#include "cameraserver/CameraServer.h"
#include "cscore_oo.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include "frc2/command/SubsystemBase.h"


namespace CONSTANTS::VISION {
    constexpr int CAMERA_WIDTH = 320,
            CAMERA_HEIGHT = 240,
            CAMERA_FPS = 20;
}

class Vision : public frc2::SubsystemBase {
public:
    enum Camera {
        INVALID = 0,
        INTAKE,
        CLIMB
    };

    Vision();

    void SwitchFeed();
    Camera GetCurrentFeed();

private:
    cs::UsbCamera climbCamera{"Climb Camera", 0},
            intakeCamera{"Intake Camera", 1};
    cs::MjpegServer videoServer{"Dashboard Server", 1185};
    cs::HttpCamera videoStream{"Dashboard Stream",
        { "http://roboRIO-2501-FRC.local/video/stream.mjpg", // mDNS
            "http://10.25.1.2/video/stream.mjpg", // Radio assigned IP
            "http://172.22.11.2/video/stream.mjpg" }, // USB IP
            // Note: The ethernet IP is not defined when not at an event.
        cs::HttpCamera::HttpCameraKind::kMJPGStreamer
    };
    Camera currentFeed = INVALID;
};
