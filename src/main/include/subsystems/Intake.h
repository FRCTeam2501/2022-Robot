#pragma once
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"
#include "frc/Motorcontrol/MotorControllerGroup.h"

#include "rev/CANPIDController.h"

#include <math.h>
#include "rev/SparkMaxLimitSwitch.h"
#include "frc/smartdashboard/SmartDashboard.h"

namespace IntakeConstants
{

  constexpr double intakeLiftSetP = 0.07;
  constexpr double intakeLiftSetI = 0.0;
  constexpr double intakeLiftSetD = 0.0;

}

class Intake : public frc2::SubsystemBase
{
public:
  Intake();

  void RollerControl(double rollerSpeed);

  void LiftControl(double liftAngle);
  void Periodic() override;
  void SetP(double pValue);
  void SetLiftEncoder(double intakeZero);

private:
  rev::CANSparkMax intakeLift{13, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxPIDController intakeLiftPID = intakeLift.GetPIDController();

  rev::SparkMaxRelativeEncoder intakeLiftEncoder = intakeLift.GetEncoder();

  rev::CANSparkMax *rollerMotor;
};
