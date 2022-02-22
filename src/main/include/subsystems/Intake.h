#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "frc/motorcontrol/MotorController.h"
#include "rev/SparkMaxLimitSwitch.h"

namespace IntakeConstants{


constexpr int Power =6;
constexpr int Updown = 7;

constexpr double updownP = 0;
constexpr double updownI = 0;
constexpr double updownD = 0;

constexpr double up = 0;
constexpr double down = 0;
constexpr double start = 0;

constexpr double inp = 0.5;
constexpr int outp = 1;

}



class Intake : public frc2::SubsystemBase {
 public:
  Intake();
~Intake();



void UpDown(double U);

void Power(double P);



  void Periodic() override;

 
 private:
 
  rev::CANSparkMax updown{IntakeConstants::Updown, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxPIDController updownPID = updown.GetPIDController();
  rev::SparkMaxRelativeEncoder pivotEncoder = updown.GetEncoder();
rev::CANSparkMax *power;
void InitDefaultCommaned();

};
