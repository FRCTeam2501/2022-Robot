#include "subsystems/Climber.h"

using namespace frc;

Climber::Climber()
{

    pivotClimb.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);
    pivotClimb.SetSmartCurrentLimit(ClimbConstants::pivotClimbSmartCurrentLimet);
    pivotClimb.SetSecondaryCurrentLimit(ClimbConstants::pivotClimbSeccondaryCurrentLimet);
    pivotClimb.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);

    pivotPID.SetP(ClimbConstants::pivotClimbSetP);
    pivotPID.SetI(ClimbConstants::pivotClimbSetP);
    pivotPID.SetD(ClimbConstants::pivotClimbSetP);
    pivotPID.SetOutputRange(-1, 1);
    pivotClimb.GetEncoder().SetPositionConversionFactor(
        (360 / (ClimbConstants::pivotConversionFactorOne * ClimbConstants::pivotConversionFactorTwo)));
    // Makes it so that one unit into the motor makes one degree of rotation of the climb arm

    winchPID.SetP(0.01);
    winchPID.SetI(0);
    winchPID.GetD(0);
    winchPID.SetOutputRange(-1, 1);

    winch.SetSmartCurrentLimit(ClimbConstants::winchSmartCurrentLimet);
    winch.SetSecondaryCurrentLimit(ClimbConstants::winchSeccondaryCurrentLimet);
    winch.GetEncoder().SetPositionConversionFactor(1/48); // not currect, but maby is
}

int Climber::ClimbControl(double angleAdjust, double lengthAdjust)
{

    seccondaryMove = false;
    lengthChanged = false;
    /*
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
    */
    length = lengthAdjust;
    angle = angleAdjust;
    winchPID.SetReference(Climber::LengthToTurns(length), rev::CANSparkMaxLowLevel::ControlType::kPosition);
    pivotPID.SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    return lengthChanged;
}

void Climber::HardLength(double floatTest){
    winchPID.SetReference(floatTest, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

int Climber::LengthToTurns(double inchesToTurns)
{

    constexpr double pi = 3.141592653589793;

    constexpr double d = 0.03;
    constexpr double l = 28;

    constexpr double d0 = 1.4275;
    constexpr double c0 = d0 * pi;
    constexpr double c1 = (d0 + 1 * d) * pi;
    constexpr double c2 = (d0 + 2 * d) * pi;
    constexpr double c3 = (d0 + 3 * d) * pi;
    constexpr double c4 = (d0 + 4 * d) * pi;
    constexpr double c5 = (d0 + 5 * d) * pi;
    constexpr double c6 = (d0 + 6 * d) * pi;
    constexpr double l1 = (l - c0);
    constexpr double l2 = (l1 - c1);
    constexpr double l3 = (l2 - c2);
    constexpr double l4 = (l3 - c3);
    constexpr double l5 = (l4 - c4);
    constexpr double l6 = (l5 - c5);

    constexpr double f6 = ((1 / c5) * l5);
    constexpr double f5 = ((1 / c4) * (l4 - l5) + f6);
    constexpr double f4 = ((1 / c3) * (l3 - l4) + f5);
    constexpr double f3 = ((1 / c2) * (l2 - l3) + f4);
    constexpr double f2 = ((1 / c1) * (l1 - l2) + f3);
    constexpr double f1 = ((1 / c0) * (l - l1) + f2);

    double maxLength = 28;
    double turns;

    if (inchesToTurns <= l5)
    {
        turns = ((1 / c5) * inchesToTurns);
    }

    if (l5 < inchesToTurns && inchesToTurns <= l4)
    {
        turns = ((1 / c4) * (inchesToTurns - l5) + f6);
    }

    if (l4 < inchesToTurns && inchesToTurns <= l3)
    {
        turns = ((1 / c3) * (inchesToTurns - l4) + f5);
    }

    if (l3 < inchesToTurns && inchesToTurns <= l2)
    {
        turns = ((1 / c2) * (inchesToTurns - l3) + f4);
    }

    if (l2 < inchesToTurns && inchesToTurns <= l1)
    {
        turns = ((1 / c1) * (inchesToTurns - l2) + f3);
    }

    if (l1 < inchesToTurns && inchesToTurns <= l)
    {
        turns = ((1 / c0) * (inchesToTurns - l1) + f2);
    }

    return turns;
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
    /*
    // This checks if we have a scedjuled seccond move once we have reached the angle we were going for
    if (seccondaryMove = true && (abs(pivotClimb->GetEncoder().GetPosition() - targetAngle) < 1))
    {
        winch.GetPIDController().SetReference(Climber::LengthToTurns(targetLength), rev::CANSparkMaxLowLevel::ControlType::kPosition);
        seccondaryMove = false;
    }
    */
}

void Climber::Init()
{
}

void Climber::InitDefaultCommand()
{
}
