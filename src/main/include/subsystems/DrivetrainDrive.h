// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "frc/drive/DifferentialDrive.h"
#include "rev/CANSparkMax.h"
#include "frc/motorcontrol/MotorController.h"
#include "frc/motorcontrol/MotorControllerGroup.h"
class DrivetrainDrive : public frc2::SubsystemBase {
 public:
  DrivetrainDrive();
~DrivetrainDrive();

void ArcadeDrive(double x, double z);

void WinchB(double r);

  void Periodic() override;

 
 private:
 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
frc::DifferentialDrive *drive;
frc::MotorControllerGroup *Left, *Right;
rev::CANSparkMax *frontL, *frontR, *rearL, *rearR,*Winch;
void InitDefaultCommaned();

};
