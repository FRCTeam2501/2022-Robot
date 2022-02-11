#include "subsystems/Climber.h"

using namespace frc;

Climber::Climber()
{
    winch = new rev::CANSparkMax(ClimbConstants::winch, rev::CANSparkMax::MotorType::kBrushless);
    pivotClimb = new rev::CANSparkMax(ClimbConstants::climbPivot, rev::CANSparkMax::MotorType::kBrushless);

    pivotClimb->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);
    pivotClimb->SetSmartCurrentLimit(ClimbConstants::pivotClimbSmartCurrentLimet);
    pivotClimb->SetSecondaryCurrentLimit(ClimbConstants::pivotClimbSeccondaryCurrentLimet);
    pivotClimb->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);

    pivotClimb->GetPIDController().SetP(ClimbConstants::pivotClimbSetP);
    pivotClimb->GetPIDController().SetI(ClimbConstants::pivotClimbSetP);
    pivotClimb->GetPIDController().SetD(ClimbConstants::pivotClimbSetP);
    pivotClimb->GetPIDController().SetOutputRange(-1, 1);
    pivotClimb->GetEncoder().SetPositionConversionFactor(
        (360 / (ClimbConstants::pivotConversionFactorOne * ClimbConstants::pivotConversionFactorTwo)));
    // Makes it so that one unit into the motor makes one degree of rotation of the climb arm

    winch->SetSmartCurrentLimit(ClimbConstants::winchSmartCurrentLimet);
    winch->SetSecondaryCurrentLimit(ClimbConstants::winchSeccondaryCurrentLimet);

    winch->GetPIDController().SetP(ClimbConstants::winchSetP);
    winch->GetPIDController().SetI(ClimbConstants::winchSetP);
    winch->GetPIDController().SetD(ClimbConstants::winchSetP);
    winch->GetPIDController().SetOutputRange(-1, 1);
    winch->GetEncoder().SetPositionConversionFactor((1 / 100)); // not currect, but maby is
}

Climber::~Climber()
{
    delete winch;
    delete pivotClimb;
}

int Climber::ClimbControl(double angleAdjust, double lengthAdjust)
{
    seccondaryMove = false;
    lengthChanged = false;

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

    if (angleAdjust > 1)
    {
        // checks new climber position to make shure that it is legal.
        if (lengthAdjust < (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos((angleAdjust * ClimbConstants::pi / (180)))) / (std::sin((angleAdjust * ClimbConstants::pi / (180)))))))
        {
            // if the length is not legal, set it to the legal length.
            lengthAdjust = (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos((angleAdjust * ClimbConstants::pi / (180)))) / (std::sin((angleAdjust * ClimbConstants::pi / (180))))));
            lengthChanged = true;
        }
    }
    // Checks if the sceleing is veing violated, if so, it will change the length to make it legal
    if (lengthAdjust > (((ClimbConstants::defaultScealing - ClimbConstants::rotationBigOffset * std::sin((angleAdjust * ClimbConstants::pi / (180)))) / std::cos((angleAdjust * ClimbConstants::pi / (180)))) - ClimbConstants::minExtension))
    {
        lengthAdjust = (((ClimbConstants::defaultScealing - ClimbConstants::rotationBigOffset * std::sin((angleAdjust * ClimbConstants::pi / (180)))) / std::cos((angleAdjust * ClimbConstants::pi / (180)))) - ClimbConstants::minExtension);
        lengthChanged = true;
    }

    if ((angleAdjust <= 15 && angleAdjust >= 6) || (angleAdjust > 15 && angle < 6) || (angleAdjust < 6 && angle > 15))
    {
        if (angleAdjust <= 15 && angleAdjust >= 6)
        {
            if (lengthAdjust < 4)
            {
                lengthAdjust = 4;
            }
        }
        else
        {
            if (lengthAdjust < 4)
            {
                targetLength = lengthAdjust;
                targetAngle = angleAdjust;
                lengthAdjust = 4;
                lengthChanged = true;
                seccondaryMove = true;
            }
        }
    }

    length = lengthAdjust;
    angle = angleAdjust;
    winch->GetPIDController().SetReference(length, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    pivotClimb->GetPIDController().SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    return lengthChanged;
}

int Climber::GetAngle()
{
    return angle;
}

int Climber::GetLength()
{
    return length;
}

void Climber::Periodic()
{
    // This checks if we have a scedjuled seccond move once we have reached the angle we were going for
    if (seccondaryMove = true && (abs(pivotClimb->GetEncoder().GetPosition() - targetAngle) < 1))
    {
        winch->GetPIDController().SetReference(targetLength, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        seccondaryMove = false;
    }
}

void Climber::InitDefaultCommand()
{
}
