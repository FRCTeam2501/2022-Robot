#pragma once
///*
#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "frc/motorcontrol/MotorController.h"
#include <frc/DigitalInput.h>

#include "units/angle.h"
#include "rev/CANPIDController.h"
#include "Constants.h"
#include <math.h> 
#include <frc/DigitalInput.h>





namespace Climerlimits{



constexpr double rasheo1 = 100;
constexpr double rasheo2 = (122/65);

}

class Climber : public frc2::SubsystemBase {
 public:
 
  Climber();
~Climber();

void clim(double A);
void two(double B);

 void Periodic() override;

 private:
 

rev::CANSparkMax *motorA,*motorL;
void InitDefaultCommaned();
};
//*/
