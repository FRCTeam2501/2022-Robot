// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//done
#include "subsystems/DrivetrainDrive.h"
#include "Constants.h"
using namespace frc; 



void DrivetrainDrive::Periodic() {}

DrivetrainDrive::DrivetrainDrive() {
frontL = new rev::CANSparkMax(Constants::FrontLeft, rev::CANSparkMax::MotorType::kBrushless);
frontR = new rev::CANSparkMax(Constants::FrontRight, rev::CANSparkMax::MotorType::kBrushless);
rearL = new rev::CANSparkMax(Constants::RearLeft, rev::CANSparkMax::MotorType::kBrushless);
rearR = new rev::CANSparkMax(Constants::RearRight, rev::CANSparkMax::MotorType::kBrushless);

Winch = new rev::CANSparkMax(Constants::FrontLeft, rev::CANSparkMax::MotorType::kBrushless);

Left = new frc::MotorControllerGroup(*frontL,*rearL);
Right = new frc::MotorControllerGroup(*frontR,*rearR);

drive = new DifferentialDrive(*Left,*Right);
}


DrivetrainDrive::~DrivetrainDrive(){
delete frontL;
delete frontR;
delete rearL;
delete rearR;
delete drive;
delete Left;
delete Right;
delete Winch;
}


void DrivetrainDrive::ArcadeDrive(double x, double z) {
drive->ArcadeDrive(x,z);
}


void DrivetrainDrive::WinchB(double r){


}


void DrivetrainDrive::InitDefaultCommaned(){

}



