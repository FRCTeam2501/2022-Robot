#include "subsystems/Climber.h"


Climber::Climber() {
    extend = new ClimberExtend();
    rotate = new ClimberRotate();
}

Climber::~Climber() {
    delete extend;
    delete rotate;
}

units::meter_t Climber::GetExtension() {
    return extend->GetExtension();
}

void Climber::SetExtension(units::meter_t distance) {
    extend->SetExtension(distance);
}

units::degree_t Climber::GetAngle() {
    return rotate->GetAngle();
}

void Climber::SetAngle(units::degree_t angle) {
    rotate->SetAngle(angle);
}

