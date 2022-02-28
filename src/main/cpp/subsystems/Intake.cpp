#include "subsystems/Intake.h"

using namespace frc;

Intake::Intake()
{

    rollerMotor = new rev::CANSparkMax(16, rev::CANSparkMax::MotorType::kBrushed);

    //  intakeLift.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);
    intakeLift.SetSmartCurrentLimit(60);
    intakeLift.SetSecondaryCurrentLimit(80);
    //  intakeLift.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);

    intakeLiftPID.SetP(IntakeConstants::intakeLiftSetP);
    intakeLiftPID.SetI(IntakeConstants::intakeLiftSetI);
    intakeLiftPID.SetD(IntakeConstants::intakeLiftSetD);
    intakeLiftPID.SetOutputRange(-0.1, 1.0);
    intakeLiftEncoder.SetPositionConversionFactor(1);
    intakeLift.SetInverted(false);

    rollerMotor->SetSmartCurrentLimit(60);
    rollerMotor->SetSecondaryCurrentLimit(80);
}

void Intake::SetP(double pValue)
{
    intakeLiftPID.SetP(pValue);
    frc::SmartDashboard::PutNumber("Lift P valuel", pValue);
}

void Intake::RollerControl(double rollerSpeed)
{

    if (rollerSpeed > 1)
    {
        rollerSpeed = 1;
    }

    if (rollerSpeed < -1)
    {
        rollerSpeed = -1;
    }

    rollerMotor->Set(rollerSpeed);
}

void Intake::LiftControl(double liftAngle)
{
    frc::SmartDashboard::PutNumber("Lift target", liftAngle);
    intakeLiftPID.SetReference(liftAngle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void Intake::SetLiftEncoder(double intakeZero)
{
    intakeLiftEncoder.SetPosition(intakeZero);
}

void Intake::Periodic()
{

    frc::SmartDashboard::PutNumber("lift Actual", intakeLiftEncoder.GetPosition());
}