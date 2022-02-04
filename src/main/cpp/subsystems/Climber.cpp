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



void Climber::WinchControl(double lengthAdjust){
    //makes shure length is not outside of limet
    if(length > 20){
        length = 20;
    }
    if(length < 0){
        length = 0;
    }

    //makes shure we are not trying to set length to a value that is outside the length limet
    if((length + lengthAdjust) > 20){
        lengthAdjust = (20 - length);
    }
    if((length + lengthAdjust) < 0){
        lengthAdjust = (length*-1);
    }
    //checks if new length is legal and if not sets length to maximum it can and sets length adjust to zero
    if((length+lengthAdjust) < (40-(23-5*std::cos(angle))/(std::sin(angle)))){
        length = (40-(23-5*std::cos(angle))/(std::sin(angle)));

        lengthAdjust = 0;
    }
    //sets length to new length
    length = (lengthAdjust + length);
    lengthAdjust = 0;
    //positions motor to new length
    winch->GetPIDController().SetReference(length, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}


void Climber::AngleControl(double angleAdjust)
{
//makes shure that angle is not outside of limet
    if(angle > 50){
    angle = 50;
    }
    if(angle < 0){
    angle = 0;
    }

    //This makes shure that the angle adjustment does not move the climber outside of the limet
    if((angle + angleAdjust) > 50){
        angleAdjust = (50 - angle);
    }
    if((angle + angleAdjust) < 0){
        angleAdjust = (angle*-1);
    }

    //combines angle and angle adjust
    angle = (angleAdjust + angle);

    //checks new climber position to make shure that it is legal. 
    if(length < (40-(23-5*std::cos(angle))/(std::sin(angle)))){
        // if the length is not legal, set it to the legal length.
        length = (40-(23-5*std::cos(angle))/(std::sin(angle)));
        winch->GetPIDController().SetReference(length, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    }
   
    angleAdjust = 0;

    pivotClimb->GetPIDController().SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void Climber::Periodic() {}

void Climber::InitDefaultCommand()
{
}
