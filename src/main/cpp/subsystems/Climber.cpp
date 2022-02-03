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

    winch->GetPIDController().SetP(0.00);
    winch->GetPIDController().SetI(0.00);
    winch->GetPIDController().SetD(0.00);
    winch->GetPIDController().SetOutputRange(-1, 1);
    winch->GetEncoder().SetPositionConversionFactor((1/100));

}

Climber::~Climber()
{
    delete winch;
    delete pivotClimb;
}

/*
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
*/



void Climber::WinchControl(double lengthAdjust){

    if((length + lengthAdjust) > 20){
        lengthAdjust = 20;
    }
    if((length + lengthAdjust) < 0){
        lengthAdjust = 0;
    }

    if((length+lengthAdjust)<(40-((23)/(std::sin(angle))))){
    length = (length+lengthAdjust);
    lengthAdjust  = 0;
    }

    winch->GetPIDController().SetReference(length, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}


void Climber::AngleControl(double angleAdjust)
{

    if(angle > 50){
    angle = 50;
    }
    if(angle < 0){
    angle = 0;
    }

    if((angle + angleAdjust) > 50){
        angleAdjust = (50 - angle);
    }
    if((angle + angleAdjust) < 0){
        angleAdjust = (angleAdjust*-1);
    }

 angle = (angleAdjust + angle);

    if(length < (40-((23)/(std::sin(angle))))){
        length = (40-((23)/(std::sin(angle))));
    }
   
    angleAdjust = 0;

    pivotClimb->GetPIDController().SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void Climber::Periodic() {}

void Climber::InitDefaultCommand()
{
}
