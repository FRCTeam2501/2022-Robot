#include "subsystems/Climber.h"

using namespace frc;

Climber::Climber()
{

    pivotClimb.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);
    pivotClimb.SetSmartCurrentLimit(ClimbConstants::pivotClimbSmartCurrentLimet);
    pivotClimb.SetSecondaryCurrentLimit(ClimbConstants::pivotClimbSeccondaryCurrentLimet);
    pivotClimb.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);

    winch.SetInverted(true);

    pivotPID.SetP(ClimbConstants::pivotClimbSetP);
    pivotPID.SetI(ClimbConstants::pivotClimbSetI);
    pivotPID.SetD(ClimbConstants::pivotClimbSetD);
    pivotPID.SetOutputRange(-1, 1);
    pivotClimb.GetEncoder().SetPositionConversionFactor(
        (360 / (ClimbConstants::pivotConversionFactorOne * ClimbConstants::pivotConversionFactorTwo)));
    // Makes it so that one unit into the motor makes one degree of rotation of the climb arm

    winchPID.SetP(2.0);
    winchPID.SetI(0.0);
    winchPID.GetD(0);
    winchPID.SetOutputRange(-1.0, 1.0);

    winch.SetSmartCurrentLimit(ClimbConstants::winchSmartCurrentLimet);
    winch.SetSecondaryCurrentLimit(ClimbConstants::winchSeccondaryCurrentLimet);
    winchEncoder.SetPositionConversionFactor( (1.4275 * M_PI)/100.0); // not currect, but maby is
}

void Climber::HardLength(double floatTest)
{
    winchPID.SetReference(floatTest, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    frc::SmartDashboard::PutNumber("Winch target", floatTest);
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
    frc::SmartDashboard::PutNumber("winch actual", winchEncoder.GetPosition());
}