#pragma once
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"
#include "frc/Motorcontrol/MotorControllerGroup.h"

#include "rev/CANPIDController.h"

#include <math.h>
#include "rev/SparkMaxLimitSwitch.h"
#include "frc/smartdashboard/SmartDashboard.h"

//The namespace for intake subsystem to keep the constants here
namespace IntakeConstants
{
//constexpr to avoid linker error
  constexpr double intakeLiftSetP = 0.07;
  constexpr double intakeLiftSetI = 0.0;
  constexpr double intakeLiftSetD = 0.0;

}

class Intake : public frc2::SubsystemBase
{
public:
//we do not need a deconstructor because we do not use pointers for this subsyste becasue REV robotics libraries
  Intake();

  void RollerControl(double rollerSpeed);

  void LiftControl(double liftAngle);
  void Periodic() override;
  void SetP(double pValue);
  void SetLiftEncoder(double intakeZero);

private:

//old way of doing it because rev
  rev::CANSparkMax intakeLift{13, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxPIDController intakeLiftPID = intakeLift.GetPIDController();

  rev::SparkMaxRelativeEncoder intakeLiftEncoder = intakeLift.GetEncoder();

  rev::CANSparkMax *rollerMotor;
};


/*
ok so this subsystem needs a bit of explenation, we used PID control in order to control the angle of the intake accuratly
But because rev robotics changed their programming libraries from 2020, the way that we did it back then did not work this year for what ever reason
So we have to do it the old way, without pointers. This was kinda how they did it back in Tyler's day

*/