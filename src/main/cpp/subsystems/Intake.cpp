#include "subsystems/Intake.h"

using namespace frc;



Intake::Intake()
{

    rollerMotor = new rev::CANSparkMax(14, rev::CANSparkMax::MotorType::kBrushless);

    intakeLift.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);
    intakeLift.SetSmartCurrentLimit(60);
    intakeLift.SetSecondaryCurrentLimit(80);
    intakeLift.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);


    intakeLiftPID.SetP(IntakeConstants::intakeLiftSetP);
    intakeLiftPID.SetI(IntakeConstants::intakeLiftSetI);
    intakeLiftPID.SetD(IntakeConstants::intakeLiftSetD);
    intakeLiftPID.SetOutputRange(-1, 1);
    intakeLiftEncoder.SetPositionConversionFactor(1/48);


    rollerMotor->SetSmartCurrentLimit(60);
    rollerMotor->SetSecondaryCurrentLimit(80);

}

void Intake::RollerControl(double rollerSpeed){

    if(rollerSpeed > 1){
        rollerSpeed = 1;
    }

    if(rollerSpeed < -1){
        rollerSpeed = -1;
    } 

    rollerMotor->Set(rollerSpeed);
}

void Intake::LiftControl(double liftAngle){

    intakeLiftPID.SetReference(liftAngle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}
