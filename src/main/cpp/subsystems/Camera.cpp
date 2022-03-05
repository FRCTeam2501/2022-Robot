#include "subsystems/Camera.h"

Camera::Camera()
{
    //ok so this year we needed to have 2 difrent cameras to switch between, (even though I only used the intake cam once and never
    //switched to climb cam during a match), we need to switch between them to avoid video bandwith limits

    //here is the camera objects, you pass it what you want the pop up menue to be called in smart dashboard(I never figured out how to do it in shuffle board), and the usb port number
    climbCamera = new cs::UsbCamera("climb cam", 0);
    intakeCamera = new cs::UsbCamera("intake cam", 1);

    //Here is the streamVodeo pointer, I have no idea what that 1185 number is, it was in Brian's (2020 programming captian), code that I took inspiration from to make this
    streamVideo = new cs::MjpegServer("Dashboard Stream", 1185);


    //Here is where we set up the cameras
    climbCamera->SetResolution(Cameras::with, Cameras::height);
    climbCamera->SetFPS(Cameras::fps);
    climbCamera->SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

    intakeCamera->SetResolution(Cameras::with, Cameras::height);
    intakeCamera->SetFPS(Cameras::fps);
    intakeCamera->SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);


    //Here is where we set up the vodeo stream
    streamVideo->SetResolution(Cameras::with, Cameras::height);
    streamVideo->SetFPS(Cameras::fps);

    //Here we start the camera stream to put it on smart dashboard
    streamVideo->SetSource(*climbCamera);
    currentFeed = 2;
    frc::CameraServer::StartAutomaticCapture();
}

Camera::~Camera()
{
    //We delete the pointers
    delete climbCamera;
    delete intakeCamera;
    delete streamVideo;
}

void Camera::SwitchFeed()
{
//This is the function that we call to switch between the cameras and put that camera to the smart dashboard stream
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
