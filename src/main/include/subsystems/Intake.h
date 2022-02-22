#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "frc/motorcontrol/MotorController.h"
#include "rev/SparkMaxLimitSwitch.h"

class Intake : public frc2::SubsystemBase {
 public:
  Intake();
~Intake();



void UpDown(double U);

void Power(double P);



  void Periodic() override;

 
 private:
 

rev::CANSparkMax *power,*updown;
void InitDefaultCommaned();

};
