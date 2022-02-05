#pragma once
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"
#include "frc/Motorcontrol/MotorControllerGroup.h"

class Climber : public frc2::SubsystemBase
{
public:
    Climber();
    ~Climber();

    void WinchesUp(double winchPowerUp);
    void WinchesDown(double winchPowerDown);
    void WinchesOff(double winchPowerOff);
    void ControlPivot(double pivotPower);



    void Periodic();
    // It's desirable that everything possible under private except
    // for methods that implement subsystem capabilities

private:
    frc::MotorControllerGroup *winches;
    rev::CANSparkMax *winchLeft, *winchRight, *pivotClimb;
    
    void InitDefaultCommand();
};