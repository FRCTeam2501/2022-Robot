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
    // lengthAdjust is the new length that we want to set the arms to
    //  makes sure length is not outside of limet
    if (length > ClimbConstants::maxLength)
    {
        length = ClimbConstants::maxLength;
    }
    if (length < ClimbConstants::minLength)
    {
        length = ClimbConstants::minLength;
    }

    // makes shure we are not trying to set length to a value that is outside the length limet
    if (lengthAdjust > ClimbConstants::maxLength)
    {
        lengthAdjust = ClimbConstants::maxLength;
    }
    if (lengthAdjust < ClimbConstants::minLength)
    {
        lengthAdjust = ClimbConstants::minLength;
    }
    // checks if new length is legal and if not sets length to maximum it can and sets length adjust to zero
 
    if (angle > 1)
    {
        if (lengthAdjust < (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos((angle / (180 * ClimbConstants::pi)))) / (std::sin((angle / (180 * ClimbConstants::pi)))))))
        {
            lengthAdjust = (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos((angle / (180 * ClimbConstants::pi)))) / (std::sin((angle / (180 * ClimbConstants::pi))))));
        }
        if (lengthAdjust > (((ClimbConstants::defaultScealing - ClimbConstants::rotationOffset * std::sin((angle / (180 * ClimbConstants::pi)))) / std::cos((angle / (180 * ClimbConstants::pi)))) - ClimbConstants::minExtension))
        {
            lengthAdjust = (((ClimbConstants::defaultScealing - ClimbConstants::rotationOffset * std::sin((angle / (180 * ClimbConstants::pi)))) / std::cos((angle / (180 * ClimbConstants::pi)))) - ClimbConstants::minExtension);
        }
    }

    // sets length to new length
    length = lengthAdjust;
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
    if (angleAdjust > ClimbConstants::maxAngle)
    {
        angleAdjust = (ClimbConstants::maxAngle);
    }
    if (angleAdjust < ClimbConstants::minAngle)
    {
        angleAdjust = ClimbConstants::minAngle;
    }

    // combines angle and angle adjust
    angle = angleAdjust;

    // Checks if angle is greater than one, this is so that there is no issue with it deviding by zero
    if (angle > 1)
    {
        // checks new climber position to make shure that it is legal.
        if (length < (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos((angle / (180 * ClimbConstants::pi)))) / (std::sin((angle / (180 * ClimbConstants::pi)))))))
        {
            // if the length is not legal, set it to the legal length.
            length = (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos((angle / (180 * ClimbConstants::pi)))) / (std::sin((angle / (180 * ClimbConstants::pi))))));
            winch->GetPIDController().SetReference(length, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        }
        if (length > (((ClimbConstants::defaultScealing - ClimbConstants::rotationOffset * std::sin((angle / (180 * ClimbConstants::pi)))) / std::cos((angle / (180 * ClimbConstants::pi)))) - ClimbConstants::minExtension))
        {
            length = (((ClimbConstants::defaultScealing - ClimbConstants::rotationOffset * std::sin((angle / (180 * ClimbConstants::pi)))) / std::cos((angle / (180 * ClimbConstants::pi)))) - ClimbConstants::minExtension);
            lengthAdjust = 0;
            winch->GetPIDController().SetReference(length, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        }
    }

    angleAdjust = 0;

    pivotClimb->GetPIDController().SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void Climber::Periodic() {}

void Climber::InitDefaultCommand()
{
}
