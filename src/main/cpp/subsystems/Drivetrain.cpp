#include "subsystems/Drivetrain.h"


void Drivetrain::ArcadeDrive(double xSpeed, double zRotation) {
    if(isInverted)
        xSpeed *= -1.0;

    drive.ArcadeDrive(xSpeed, zRotation);
}

bool Drivetrain::IsInverted() {
    return isInverted;
}

void Drivetrain::SetInverted(bool inverted) {
    isInverted = inverted;
}