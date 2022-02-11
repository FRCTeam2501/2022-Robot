#include "subsystems/Camera.h"



void Camera::Camera(){
climbCamera = new cs::UsbCamera ("climb cam", 0);
intakeCamera = new cs::UsbCamera ("intake cam", 1);

 streamVideo = new cs::MjpegServer("Dashboard Stream", 1185);



    climbCamera->SetResolution(Cameras::with, Cameras::height);
	climbCamera->SetFPS(Cameras::fps);
	climbCamera->SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

    intakeCamera->SetResolution(Cameras::with, Cameras::height);
	intakeCamera->SetFPS(Cameras::fps);
	intakeCamera->SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

    streamVideo->SetResolution(Cameras::with, Cameras::height);
	streamVideo->SetFPS(Cameras::fps);
    
}

void Camera::~Camera(){
     delete climbCamera;
     delete intakeCamera;
     delete streamVideo;
}

void Camera::SwitchFeed(){

if (currentFeed = 1){
    currentFeed = 2;
}else{
    currentFeed = 1;
}

    switch (currentFeed)
    {
    case 1:
    streamVideo->SetSource(*climbCamera);
        break;
    case 2:
    streamVideo->SetSource(*intakeCamera);
    break; 

    default:
        break;
    }
}

void Camera::Periodic(){

}

void Camera::init(){
    currentFeed = 1; 
    streamVideo->SetSource(*climbCamera);
}





