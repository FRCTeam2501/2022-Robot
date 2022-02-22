#include "subsystems/Intake.h"
#include "Constants.h"


using namespace frc; 



void Intake::Periodic() {}

Intake::Intake() {
power = new rev::CANSparkMax(IntakeConstants::Power, rev::CANSparkMax::MotorType::kBrushless);
updown = new rev::CANSparkMax(IntakeConstants::Updown, rev::CANSparkMax::MotorType::kBrushless);



 updownPID.SetP(ClimbConstants::winchSetP);
    updownPID.SetI(ClimbConstants::winchSetI);
    updownPID.GetD(ClimbConstants::winchSetD);
    updownPID.SetOutputRange(-1.0, 1.0);

 powerPID.SetP(ClimbConstants::winchSetP);
    powerPID.SetI(ClimbConstants::winchSetI);
    powerPID.GetD(ClimbConstants::winchSetD);
    powerPID.SetOutputRange(-1.0, 1.0);



//updown->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);
//updown->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);

}

Intake::~Intake() {
delete power;
delete updown;
   
}

void Intake::UpDown(double U){
updown->Set(U);
}
void Intake::Power(double P){
power->Set(P);
}



void Intake::InitDefaultCommaned(){}
