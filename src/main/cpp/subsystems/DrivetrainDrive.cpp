// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/DrivetrainDrive.h"
#include "Constants.h"
using namespace frc;

  void DrivetrainDrive::Periodic(){}

 
DrivetrainDrive::DrivetrainDrive()
{
  frontLeft = new rev::CANSparkMax(Constants::FrontLeft, rev::CANSparkMax::MotorType::kBrushless);
  frontRight = new rev::CANSparkMax(Constants::FrontRight, rev::CANSparkMax::MotorType::kBrushless);
  rearLeft = new rev::CANSparkMax(Constants::RearLeft, rev::CANSparkMax::MotorType::kBrushless);
  rearRight = new rev::CANSparkMax(Constants::RearRight, rev::CANSparkMax::MotorType::kBrushless);
  rearLeft->Follow(*frontLeft);
  rearRight->Follow(*frontRight);
  drive = new DifferentialDrive(*frontLeft, *frontRight);
}
DrivetrainDrive::~DrivetrainDrive()
{
  delete frontLeft;
  delete frontRight;
  delete rearLeft;
  delete rearRight;
  delete drive;
}
void DrivetrainDrive::ArcadeDrive(double x, double z){
    
        
    drive->ArcadeDrive(x, z);
}
void DrivetrainDrive::InitDefaultCommand()
{
  
}
// Put methods for controlling this subsystem
// here. Call these from Commands.