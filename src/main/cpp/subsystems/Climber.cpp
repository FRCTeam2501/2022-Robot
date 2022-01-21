#include "subsystems/Climber.h"
#include "Constants.h"

using namespace frc;

Climber::Climber()
{
    winchLeft = new rev::CANSparkMax(Constants::winchLeft, rev::CANSparkMax::MotorType::kBrushless);
    winchRight = new rev::CANSparkMax(Constants::winchRight, rev::CANSparkMax::MotorType::kBrushless);
    pivotClimb = new rev::CANSparkMax(Constants::climbPivot, rev::CANSparkMax::MotorType::kBrushless);

    winches = new frc::MotorControllerGroup(*winchLeft, *winchRight);

    
}

Climber::~Climber()
{
    delete winchLeft;
    delete winchRight;
    delete pivotClimb;

    delete winches;

}

void Climber::WinchesUp(double winchPowerUp){
    winches->Set(winchPowerUp);
}

void Climber::WinchesDown(double winchPowerDown){
    winches->Set(winchPowerDown);
}

void Climber::WinchesOff(double winchPowerOff){
    winches->Set(winchPowerOff);
}

void Climber::ControlPivot(double pivotPower){
    pivotClimb->Set(pivotPower);
}
void Climber::Periodic() {}

void Climber::InitDefaultCommand()
{
}
