// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/DrivetrainDrive.h"
#include "Constants.h"
using namespace frc;

void DrivetrainDrive::Periodic() {}

DrivetrainDrive::DrivetrainDrive()
{
  //This is where we create the objects to the motor controllers that control the tank drive drivetrain
  //The first part in the perencicies is the can ID, this can be set by connecting to the motor controllers with the 
  //Rev motor controller ap and burning the flash to the new can ID
  frontLeft = new rev::CANSparkMax(Drivetrain::frontLeft, rev::CANSparkMax::MotorType::kBrushless);
  frontRight = new rev::CANSparkMax(Drivetrain::frontRight, rev::CANSparkMax::MotorType::kBrushless);
  rearLeft = new rev::CANSparkMax(Drivetrain::rearLeft, rev::CANSparkMax::MotorType::kBrushless);
  rearRight = new rev::CANSparkMax(Drivetrain::rearRight, rev::CANSparkMax::MotorType::kBrushless);

//We set current limets to not overexert the motors and not depleat the battery. 
  frontLeft->SetSmartCurrentLimit(Drivetrain::driveSmartCurrentLimet);
  frontLeft->SetSecondaryCurrentLimit(Drivetrain::driveSeccondaryCurrentLimet);
  frontLeft->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  frontRight->SetSmartCurrentLimit(Drivetrain::driveSmartCurrentLimet);
  frontRight->SetSecondaryCurrentLimit(Drivetrain::driveSeccondaryCurrentLimet);
  frontRight->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  rearLeft->SetSmartCurrentLimit(Drivetrain::driveSmartCurrentLimet);
  rearLeft->SetSecondaryCurrentLimit(Drivetrain::driveSeccondaryCurrentLimet);
  rearLeft->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  rearRight->SetSmartCurrentLimit(Drivetrain::driveSmartCurrentLimet);
  rearRight->SetSecondaryCurrentLimit(Drivetrain::driveSeccondaryCurrentLimet);
  rearRight->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  //This is where we invert the motors to make the motors move in unison corectly
  frontLeft->SetInverted(false);
  frontRight->SetInverted(true);

//We create a motor controller group object to control the left drive motors and right drive motors together. 
  LeftSide = new frc::MotorControllerGroup(*frontLeft, *rearLeft);
  RightSide = new frc::MotorControllerGroup(*frontRight, *rearRight);

//we create a differential drive object to control the drivetrain
  drive = new DifferentialDrive(*LeftSide, *RightSide);
  RightSide->SetInverted(false);
}
DrivetrainDrive::~DrivetrainDrive()
{
  //delete pointers
  delete frontLeft;
  delete frontRight;
  delete rearLeft;
  delete rearRight;
  delete drive;
  delete LeftSide;
  delete RightSide;
}

void DrivetrainDrive::DriveControl(double l, double r)
{
  //this function was never used, opps
  //all part of the process
  LeftSide->Set(l);
  RightSide->Set(r);
}

void DrivetrainDrive::ArcadeDrive(double x, double z)
{
//Here we pass the values to the arcadedrive function in the DifferentialDrive.cpp
  drive->ArcadeDrive(x, z);
}
void DrivetrainDrive::InitDefaultCommand()
{
}
// Put methods for controlling this subsystem
// here. Call these from Commands.