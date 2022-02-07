#pragma once
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"
#include "frc/Motorcontrol/MotorControllerGroup.h"
#include "units/angle.h"
#include "rev/CANPIDController.h"
#include "Constants.h"
#include <math.h> 

namespace ClimbConstants{

  constexpr double rotationOffset = 3;
  constexpr double defaultClimbLength = 40;
  constexpr double pivotToFrameDist = 26.5; //checked
  constexpr double maxDistFromFrame = 12;

  constexpr double maxAngle = 50;
  constexpr double minAngle = 0;
  constexpr double maxLength = 20;
  constexpr double minLength = 0;

  constexpr double defaultScealing = 25;
  constexpr double minExtension = 8;

  constexpr double pi =  3.1415926535;

  constexpr int winch = 6;
  
  constexpr int climbPivot = 8;
  
  constexpr int PivotConversionFactorOne = 100;
  constexpr int PivotConversionFactorTwo = (122/65);
}

class Climber : public frc2::SubsystemBase
{
public:
    Climber();
    ~Climber();


    void AngleControl(double angle);
   int GetAngle();
   int GetLength();
    
    void WinchControl(double lengthAdjust);

    void ClimbControl(double angleAdjust, double lengthAdjust);

    void Periodic();
    
    // It's desirable that everything possible under private except
    // for methods that implement subsystem capabilities

private:
    double angle;
    double angleAdjust;

    double length;
    double lengthAdjust;


    rev::CANSparkMax *winch, *pivotClimb;
    
    void InitDefaultCommand();
};