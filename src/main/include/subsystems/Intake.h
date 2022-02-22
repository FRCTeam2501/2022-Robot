#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "frc/motorcontrol/MotorController.h"
#include "rev/SparkMaxLimitSwitch.h"

namespace IntakeConstants{


constexpr int Power =6;
constexpr int Updown = 7;


}



class Intake : public frc2::SubsystemBase {
 public:
  Intake();
~Intake();



void UpDown(double U);

void Power(double P);



  void Periodic() override;

 
 private:
 
 rev::CANSparkMax power{IntakeConstants::Power, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax updown{IntakeConstants::Updown, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxPIDController powerPID = power.GetPIDController();
  rev::SparkMaxPIDController updownPID = updown.GetPIDController();


void InitDefaultCommaned();

};
