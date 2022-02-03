#pragma once
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"
#include "frc/Motorcontrol/MotorControllerGroup.h"
#include "units/angle.h"
#include "rev/CANPIDController.h"
#include "Constants.h"
#include <math.h> 


class Climber : public frc2::SubsystemBase
{
public:
    Climber();
    ~Climber();

    //void WinchesUp(double winchPowerUp);
    //void WinchesDown(double winchPowerDown);
    //void WinchesOff(double winchPowerOff);
    //void ControlPivot(double pivotPower);

    void AngleControl(double angle);
   
    
    void WinchControl(double LengthAdjust);
 

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