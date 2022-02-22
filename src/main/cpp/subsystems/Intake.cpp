#include "subsystems/Intake.h"
#include "Constants.h"


using namespace frc; 



void Intake::Periodic() {}

Intake::Intake() {
power = new rev::CANSparkMax(Constants::Power, rev::CANSparkMax::MotorType::kBrushless);
updown = new rev::CANSparkMax(Constants::Updown, rev::CANSparkMax::MotorType::kBrushless);

updown->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);
updown->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);

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
