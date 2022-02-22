#include "subsystems/Intake.h"
#include "Constants.h"


using namespace frc; 



void Intake::Periodic() {}

Intake::Intake() {
power = new rev::CANSparkMax(IntakeConstants::Power, rev::CANSparkMax::MotorType::kBrushed);
//updown = new rev::CANSparkMax(IntakeConstants::Updown, rev::CANSparkMax::MotorType::kBrushless);

 updownPID.SetP(IntakeConstants::updownP);
    updownPID.SetI(IntakeConstants::updownI);
    updownPID.GetD(IntakeConstants::updownD);
    updownPID.SetOutputRange(-1.0, 1.0);

}

Intake::~Intake() {
delete power;
//delete updown;
   
}

void Intake::UpDown(double U){
 updownPID.SetReference(U, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}
void Intake::Power(double P){
power->Set(P);
}



void Intake::InitDefaultCommaned(){}
