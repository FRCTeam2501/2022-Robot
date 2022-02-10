#pragma once
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"
#include "frc/Motorcontrol/MotorControllerGroup.h"
#include "units/angle.h"
#include "rev/CANPIDController.h"
#include "Constants.h"
#include <math.h> 
#include "rev/SparkMaxLimitSwitch.h"

namespace ClimbConstants{

  constexpr double rotationBigOffset = 4.5;
  //rotationBigOffset is the horisontal distance bretween the top left of the hook and the actual rotation point
  constexpr double rotationOffset = 3;
  constexpr double defaultClimbLength = 40;
  constexpr double pivotToFrameDist = 26.5; //checked
  constexpr double maxDistFromFrame = 12;

  constexpr double maxAngle = 70;
  constexpr double minAngle = 0;
  constexpr double maxLength = 20;
  constexpr double minLength = 0;

  constexpr double defaultScealing = 25;
  constexpr double minExtension = 8;

  constexpr double pi =  3.14159265358979;

  constexpr int winch = 6;
  
  constexpr int climbPivot = 8;
  
  constexpr int pivotConversionFactorOne = 100;
  constexpr int pivotConversionFactorTwo = (122/65);

  constexpr double winchSmartCurrentLimet = 60.0;
  constexpr double winchSeccondaryCurrentLimet = 70.0;

  constexpr double pivotClimbSmartCurrentLimet = 60.0;
  constexpr double pivotClimbSeccondaryCurrentLimet = 70.0;

  constexpr double pivotClimbSetP = 0.00;
  constexpr double pivotClimbSetI = 0.00;
  constexpr double pivotClimbSetD = 0.00;
  constexpr double winchSetP = 0.00;
  constexpr double winchSetI = 0.00;
  constexpr double winchSetD = 0.00;

}

class Climber : public frc2::SubsystemBase
{
public:
    Climber();
    ~Climber();


    //void AngleControl(double angle);
   int GetAngle();
   int GetLength();
    
    //void WinchControl(double lengthAdjust);

    int ClimbControl(double angleAdjust, double lengthAdjust);

    void Periodic();
    
    // It's desirable that everything possible under private except
    // for methods that implement subsystem capabilities

private:
    double angle;
    double angleAdjust;
    double targetAngle;

    double length;
    double lengthAdjust;
    double targetLength; 

    bool lengthChanged;

    bool seccondaryMove;

    rev::CANSparkMax *winch, *pivotClimb;
    
    void InitDefaultCommand();
};