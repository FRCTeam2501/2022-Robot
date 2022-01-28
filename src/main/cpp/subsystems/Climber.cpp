#include "subsystems/Climber.h"
#include "Constants.h"

using namespace frc;

Climber::Climber()
{
    winch = new rev::CANSparkMax(Constants::winch, rev::CANSparkMax::MotorType::kBrushless);
    pivotClimb = new rev::CANSparkMax(Constants::climbPivot, rev::CANSparkMax::MotorType::kBrushless);


    winch->GetPIDController().SetP(0.01);
    winch->GetPIDController().SetI(0.00);
    winch->GetPIDController().SetD(0.00);
    winch->GetPIDController().SetOutputRange(-1,1);
}

Climber::~Climber()
{
    delete winch;
    delete pivotClimb;

}

void Climber::WinchesUp(double winchPowerUp){
    winch->Set(winchPowerUp);
}

void Climber::WinchesDown(double winchPowerDown){
    winch->Set(winchPowerDown);
}

void Climber::WinchesOff(double winchPowerOff){
    winch->Set(winchPowerOff);
}

void Climber::ControlPivot(double pivotPower){
    pivotClimb->Set(pivotPower);
}

void Climber::AngleControl(units::degree_t angle){

     PivotClimb->GetPIDController().SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);

}


void Climber::Periodic() {}

void Climber::InitDefaultCommand()
{
}
