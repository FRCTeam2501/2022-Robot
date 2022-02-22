#include "subsystems/Climber.h"
#include "Constants.h"
///*
using namespace frc; 

frc::DigitalInput climerlimitswich {2};

void Climber::Periodic() {}

Climber::Climber() {
motorL = new rev::CANSparkMax(Constants::MotorL, rev::CANSparkMax::MotorType::kBrushless);
motorA = new rev::CANSparkMax(Constants::MotorA, rev::CANSparkMax::MotorType::kBrushless);

    motorA->GetPIDController().SetP(0.00);
    motorA->GetPIDController().SetI(0.00);
    motorA->GetPIDController().SetD(0.00);
    motorA->GetPIDController().SetOutputRange(-1, 1);
    motorA->GetEncoder().SetPositionConversionFactor(
        (360 / (Climerlimits::rasheo1 * Climerlimits::rasheo2)));

    motorL->GetPIDController().SetP(0.00);
    motorL->GetPIDController().SetI(0.00);
    motorL->GetPIDController().SetD(0.00);
    motorL->GetPIDController().SetOutputRange(-1, 1);
    motorL->GetEncoder().SetPositionConversionFactor((1 / 100));
}

Climber::~Climber() {
delete motorL;
delete motorA;
   
}

void Climber::clim(double A){
if(A==0){
motorL->GetPIDController().SetReference(movment0, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}
if(A==1){
    motorL->GetPIDController().SetReference(movment1, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}
if(A==2){
    motorL->GetPIDController().SetReference(movment2, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}
if(A==3){
    motorA->GetPIDController().SetReference(movment3, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}
if(A==4){
    motorL->GetPIDController().SetReference(movment4, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}
if(A==5){
    motorA->GetPIDController().SetReference(movment5, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}
if(A==6){
    motorL->GetPIDController().SetReference(movment6, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}
if(A==7){
    motorA->GetPIDController().SetReference(movment7, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

}






void Climber::InitDefaultCommaned(){}
//*/