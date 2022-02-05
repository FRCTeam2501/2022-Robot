#include "subsystems/Climber.h"

using namespace frc;

Climber::Climber()
{
    winch = new rev::CANSparkMax(ClimbConstants::winch, rev::CANSparkMax::MotorType::kBrushless);
    pivotClimb = new rev::CANSparkMax(ClimbConstants::climbPivot, rev::CANSparkMax::MotorType::kBrushless);

    pivotClimb->GetPIDController().SetP(0.00);
    pivotClimb->GetPIDController().SetI(0.00);
    pivotClimb->GetPIDController().SetD(0.00);
    pivotClimb->GetPIDController().SetOutputRange(-1, 1);
    pivotClimb->GetEncoder().SetPositionConversionFactor(
        (360 / (ClimbConstants::PivotConversionFactorOne * ClimbConstants::PivotConversionFactorTwo)));
    // Makes it so that one unit into the motor makes one degree of rotation of the climb arm

    winch->GetPIDController().SetP(0.00);
    winch->GetPIDController().SetI(0.00);
    winch->GetPIDController().SetD(0.00);
    winch->GetPIDController().SetOutputRange(-1, 1);
    winch->GetEncoder().SetPositionConversionFactor((1 / 100));
}

Climber::~Climber()
{
    delete winch;
    delete pivotClimb;
}

void Climber::WinchControl(double lengthAdjust)
{
    // makes shure length is not outside of limet
    if (length > ClimbConstants::maxLength)
    {
        length = ClimbConstants::maxLength;
    }
    if (length < ClimbConstants::minLength)
    {
        length = ClimbConstants::minLength;
    }

    // makes shure we are not trying to set length to a value that is outside the length limet
    if ((length + lengthAdjust) > ClimbConstants::maxLength)
    {
        lengthAdjust = (ClimbConstants::maxLength - length);
    }
    if ((length + lengthAdjust) < ClimbConstants::minLength)
    {
        lengthAdjust = (length * -1);
    }
    // checks if new length is legal and if not sets length to maximum it can and sets length adjust to zero

    if (angle > 1)
    {
        if ((length + lengthAdjust) < (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos(angle)) / (std::sin(angle)))))
        {
            length = (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos(angle)) / (std::sin(angle))));

            lengthAdjust = 0;
        }
    }

    // sets length to new length
    length = (lengthAdjust + length);
    lengthAdjust = 0;
    // positions motor to new length
    winch->GetPIDController().SetReference(length, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void Climber::AngleControl(double angleAdjust)
{
    // makes shure that angle is not outside of limet
    if (angle > ClimbConstants::maxAngle)
    {
        angle = ClimbConstants::maxAngle;
    }
    if (angle < ClimbConstants::minAngle)
    {
        angle = ClimbConstants::minAngle;
    }

    // This makes shure that the angle adjustment does not move the climber outside of the limet
    if ((angle + angleAdjust) > ClimbConstants::maxAngle)
    {
        angleAdjust = (ClimbConstants::maxAngle - angle);
    }
    if ((angle + angleAdjust) < ClimbConstants::minAngle)
    {
        angleAdjust = (angle * -1);
    }

    // combines angle and angle adjust
    angle = (angleAdjust + angle);

    // checks new climber position to make shure that it is legal.
    if (length < (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos(angle)) / (std::sin(angle)))))
    {
        // if the length is not legal, set it to the legal length.
        length = (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos(angle)) / (std::sin(angle))));
        winch->GetPIDController().SetReference(length, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    }

    angleAdjust = 0;

    pivotClimb->GetPIDController().SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void Climber::Periodic() {}

void Climber::InitDefaultCommand()
{
}
