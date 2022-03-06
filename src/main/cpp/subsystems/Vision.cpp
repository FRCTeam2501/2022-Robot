#include "subsystems/Vision.h"


Vision::Vision() {
    climbCamera.SetResolution(CONSTANTS::VISION::CAMERA_WIDTH,
            CONSTANTS::VISION::CAMERA_HEIGHT);
    climbCamera.SetFPS(CONSTANTS::VISION::CAMERA_FPS);
    climbCamera.SetConnectionStrategy(
            cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);

    intakeCamera.SetResolution(CONSTANTS::VISION::CAMERA_WIDTH,
            CONSTANTS::VISION::CAMERA_HEIGHT);
    intakeCamera.SetFPS(CONSTANTS::VISION::CAMERA_FPS);
    intakeCamera.SetConnectionStrategy(
            cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);

    videoServer.SetSource(intakeCamera);
    currentFeed = INTAKE;

    frc::CameraServer::StartAutomaticCapture();
}

void Vision::SwitchFeed() {
    switch(currentFeed) {
        case INTAKE:
            videoServer.SetSource(climbCamera);
            currentFeed = CLIMB;
            break;
        case CLIMB:
        case INVALID:
        default:
            videoServer.SetSource(intakeCamera);
            currentFeed = INTAKE;
            break;
    }
}

Vision::Camera Vision::GetCurrentFeed() {
    return static_cast<Camera>(currentFeed);
}
