#include "subsystems/Climber.h"


using namespace frc;

Climber::Climber()
{
    winch = new rev::CANSparkMax(Constants::winch, rev::CANSparkMax::MotorType::kBrushless);
    pivotClimb = new rev::CANSparkMax(Constants::climbPivot, rev::CANSparkMax::MotorType::kBrushless);

    pivotClimb->GetPIDController().SetP(0.00);
    pivotClimb->GetPIDController().SetI(0.00);
    pivotClimb->GetPIDController().SetD(0.00);
    pivotClimb->GetPIDController().SetOutputRange(-1, 1);
    pivotClimb->GetEncoder().SetPositionConversionFactor(
        (360 / (Constants::PivotConversionFactorOne * Constants::PivotConversionFactorTwo)));
    // Makes it so that one unit into the motor makes one degree of rotation of the climb arm
}

Climber::~Climber()
{
    delete winch;
    delete pivotClimb;
}

void Climber::WinchesUp(double winchPowerUp)
{
    winch->Set(winchPowerUp);
}

void Climber::WinchesDown(double winchPowerDown)
{
    winch->Set(winchPowerDown);
}

void Climber::WinchesOff(double winchPowerOff)
{
    winch->Set(winchPowerOff);
}

void Climber::ControlPivot(double pivotPower)
{
    pivotClimb->Set(pivotPower);
}


void Climber::AngleControl()
{

    if(Constants::angle > 50){
        Constants::angle = 50;
    }
    if(Constants::angle < 0){
        Constants::angle = 0;
    }
    pivotClimb->GetPIDController().SetReference(Constants::angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void Climber::Periodic() {}

void Climber::InitDefaultCommand()
{
}
