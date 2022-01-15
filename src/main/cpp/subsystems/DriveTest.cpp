// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTest.h"

DriveTest::DriveTest()
    : PIDSubsystem(
          // The PIDController used by the subsystem
          frc2::PIDController(0, 0, 0)) {}

void DriveTest::UseOutput(double output, double setpoint) {
  // Use the output here
}

double DriveTest::GetMeasurement() {
  // Return the process variable measurement here
  return 0;
}
