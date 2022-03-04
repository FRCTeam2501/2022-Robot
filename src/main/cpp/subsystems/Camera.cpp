#include "subsystems/Camera.h"

Camera::Camera()
{
    climbCamera = new cs::UsbCamera("climb cam", 0);
    intakeCamera = new cs::UsbCamera("intake cam", 1);
    streamVideo = new cs::MjpegServer("Dashboard Stream", 1185);

    climbCamera->SetResolution(Cameras::with, Cameras::height);
    climbCamera->SetFPS(Cameras::fps);
    climbCamera->SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

    intakeCamera->SetResolution(Cameras::with, Cameras::height);
    intakeCamera->SetFPS(Cameras::fps);
    intakeCamera->SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

    streamVideo->SetResolution(Cameras::with, Cameras::height);
    streamVideo->SetFPS(Cameras::fps);

    streamVideo->SetSource(*climbCamera);
    currentFeed = 2;
    frc::CameraServer::StartAutomaticCapture();
}

Camera::~Camera()
{
    delete climbCamera;
    delete intakeCamera;
    delete streamVideo;
}

void Camera::SwitchFeed()
{

    switch (currentFeed)
    {
    case 1:
        streamVideo->SetSource(*climbCamera);
        currentFeed = 2;
        break;
    case 2:
        streamVideo->SetSource(*intakeCamera);
        currentFeed = 1;
        break;

    default:
        break;
    }
}
