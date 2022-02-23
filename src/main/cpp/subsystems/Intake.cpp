#include "subsystems/Intake.h"
#include "Constants.h"

using namespace frc; 

void Intake::Periodic() {}

Intake::Intake() {
power = new rev::CANSparkMax(IntakeConstants::Power, rev::CANSparkMax::MotorType::kBrushed);

 updownPID.SetP(IntakeConstants::updownP);
    updownPID.SetI(IntakeConstants::updownI);
    updownPID.GetD(IntakeConstants::updownD);
    updownPID.SetOutputRange(-1.0, 1.0);   
}

Intake::~Intake() {
delete power;
}

void Intake::UpDown(double Height){
 updownPID.SetReference(Height/3.6, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void Intake::Power(double Speed){
power->Set(Speed);
}

void Intake::InitDefaultCommaned(){}
