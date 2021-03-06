#pragma once
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"
#include "frc/Motorcontrol/MotorControllerGroup.h"
#include "units/angle.h"
#include "rev/CANPIDController.h"
#include "Constants.h"
#include <math.h>
#include "rev/SparkMaxLimitSwitch.h"
#include "frc/Solenoid.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "iostream"

namespace ClimbConstants
{
  // Here are all the constants for the climber subsystem

  constexpr double rotationBigOffset = 7.0;
  // rotationBigOffset is the horisontal distance bretween the top left of the hook and the actual rotation point
  constexpr double rotationOffset = 3;
  constexpr double defaultClimbLength = 40;
  constexpr double pivotToFrameDist = 26.5; // checked
  constexpr double maxDistFromFrame = 12;

  constexpr double maxAngle = 80;
  constexpr double minAngle = -4;
  constexpr double maxLength = 27;
  constexpr double minLength = -6;

  constexpr double batteryMinAngle = 3.0;
  constexpr double batteryMaxAngle = 32.0;
  constexpr double batteryMinLength = 8.5;

  constexpr double defaultScealing = 21;
  constexpr double minExtension = 8;

  constexpr double pi = 3.14159265358979;

  constexpr int winch = 10;

  constexpr int climbPivot = 11;

  constexpr double pivotConversionFactorOne = 100.0;
  constexpr double pivotConversionFactorTwo = 2; //(122.0 / 65.0);

  constexpr double winchSmartCurrentLimet = 60.0;
  constexpr double winchSeccondaryCurrentLimet = 70.0;

  constexpr double pivotClimbSmartCurrentLimet = 60.0;
  constexpr double pivotClimbSeccondaryCurrentLimet = 70.0;

  constexpr double pivotClimbSetP = 0.07;
  constexpr double pivotClimbSetI = 0.000;
  constexpr double pivotClimbSetD = 0.00;
  constexpr double winchSetP = 2.00;
  constexpr double winchSetI = 0.00;
  constexpr double winchSetD = 0.00;

}

class Climber : public frc2::SubsystemBase
{
  // it is public in the class so that the whole code can see these and use them
public:
  Climber();

  // void AngleControl(double angle);
  double GetAngle();
  double GetLength();

  void PinOut();
  void PinIn();
  bool PinStatus();

  void DislodgeWrench();

  // void WinchControl(double lengthAdjust);

  void ArmHorizontal();
  void SwingAndClamp();

  void ClimbPivotSetEncoder(double pivotSetEncoder);
  void ClimbWinchSetEncoder(double winchSetEncoder);

  int ClimbControl(double angleAdjust, double lengthAdjust);

  double LengthToTurns(double inchesToTurns);

  void Periodic();

private:
  // These are the values that we use to keep track of things in the climber subsystem
  double angle = 0.0;
  double angleAdjust;
  double targetAngle;

  bool horizontalActivated;
  bool swingActivated;

  bool dislodgingWrench = false;
  bool wrenchDislodged = false;
  double dislodgeTarget;

  double length = 0.0;
  double lengthAdjust;
  double targetLength;

  bool lengthChanged;

  bool seccondaryMove = false;
  bool thirdMove = false;

  // This is the old way of making motor controller objects because REV robotics made it that way and I am annoid
  rev::CANSparkMax winch{ClimbConstants::winch, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax pivotClimb{ClimbConstants::climbPivot, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxPIDController winchPID = winch.GetPIDController();
  rev::SparkMaxPIDController pivotPID = pivotClimb.GetPIDController();
  rev::SparkMaxRelativeEncoder winchEncoder = winch.GetEncoder();
  rev::SparkMaxRelativeEncoder pivotEncoder = pivotClimb.GetEncoder();
  frc::Solenoid *winchPin;
  void InitDefaultCommand();
};