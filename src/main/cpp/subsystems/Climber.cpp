#include "subsystems/Climber.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "units/math.h"


Climber::Climber() {
    extend = new ClimberExtend();
    rotate = new ClimberRotate();
}

Climber::~Climber() {
    delete extend;
    delete rotate;
}

units::meter_t Climber::GetExtension() {
    return extend->Get();
}

bool Climber::SetExtension(units::meter_t extension) {
    return Set(extension, GetAngle());
}

units::degree_t Climber::GetAngle() {
    return rotate->Get();
}

bool Climber::SetAngle(units::degree_t angle) {
    return Set(GetExtension(), angle);
}

bool Climber::Set(units::meter_t extension, units::degree_t angle) {
    bool result = false;

    // Hard extension limits
    if(extension > CONSTANTS::CLIMBER::EXTENSION_MAXIMUM) {
        extension = CONSTANTS::CLIMBER::EXTENSION_MAXIMUM;
        std::cout << "Climber: constrained extension to: "
                << extension.to<double>() << "m (hard)\n";
        result = true;
    }
    else if(extension < CONSTANTS::CLIMBER::EXTENSION_MINIMUM) {
        extension = CONSTANTS::CLIMBER::EXTENSION_MINIMUM;
        std::cout << "Climber: constrained extension to: "
                << extension.to<double>() << "m (hard)\n";
        result = true;
    }

    // Hard angle limits
    if(angle > CONSTANTS::CLIMBER::ANGLE_MAXIMUM) {
        angle = CONSTANTS::CLIMBER::ANGLE_MAXIMUM;
        std::cout << "Climber: constrained angle to: "
                << angle.to<double>() << "deg (hard)\n";
        result = true;
    }
    else if(angle < CONSTANTS::CLIMBER::ANGLE_MINIMUM) {
        angle = CONSTANTS::CLIMBER::ANGLE_MINIMUM;
        std::cout << "Climber: constrained angle to: "
                << angle.to<double>() << "deg (hard)\n";
        result = true;
    }

    // Handle the battery box constraint
    hasNextStep = false;
    units::degree_t currentAngle = GetAngle();
    // Target is below the battery box maximum, and the current is above
    // aka transitioning into the battery box
    if(angle < CONSTANTS::CLIMBER::BATTERY_BOX_ANGLE_MAXIMUM
            && currentAngle > CONSTANTS::CLIMBER::BATTERY_BOX_ANGLE_MAXIMUM) {
        // If the target is below the minimum, setup a next step to extend again
        if(angle < CONSTANTS::CLIMBER::BATTERY_BOX_ANGLE_MINIMUM) {
            std::cout << "Climber: setup next step to: "
                    << extension.to<double>() << "m (down)\n";
            nextStepExtension = extension;
            hasNextStep = true;
        }
        // Constrain the extension to more than the extension minimum
        if(extension < CONSTANTS::CLIMBER::BATTERY_BOX_EXTENSION_MINIMUM) {
            extension = CONSTANTS::CLIMBER::BATTERY_BOX_EXTENSION_MINIMUM;
            std::cout << "Climber: constrained extension to: "
                    << extension.to<double>() << "m (battery box constraint)\n";
            result = true;
        }
    }
    // Target is above the battery box minimum, and the current is below
    // aka transitioning into the battery box
    else if(angle > CONSTANTS::CLIMBER::BATTERY_BOX_ANGLE_MINIMUM
            && currentAngle < CONSTANTS::CLIMBER::BATTERY_BOX_ANGLE_MINIMUM) {
        // If the target is above the maximum, setup a next step to extend again
        if(angle > CONSTANTS::CLIMBER::BATTERY_BOX_ANGLE_MAXIMUM) {
            std::cout << "Climber: setup next step to: "
                    << extension.to<double>() << "m (up)\n";
            nextStepExtension = extension;
            hasNextStep = true;
        }
        // Constrain the extension to less than the extension maximum
        if(extension > CONSTANTS::CLIMBER::BATTERY_BOX_EXTENSION_MINIMUM) {
            extension = CONSTANTS::CLIMBER::BATTERY_BOX_EXTENSION_MINIMUM;
            std::cout << "Climber: constrained extension to: "
                    << extension.to<double>() << "m (battery box constraint)\n";
            result = true;
        }
    }
    // Current is within the battery box, constrain the extension
    else if(currentAngle < CONSTANTS::CLIMBER::BATTERY_BOX_ANGLE_MAXIMUM
            && currentAngle > CONSTANTS::CLIMBER::BATTERY_BOX_ANGLE_MINIMUM) {
        // Current is in the battery box
        if(extension < CONSTANTS::CLIMBER::BATTERY_BOX_EXTENSION_MINIMUM) {
            extension = CONSTANTS::CLIMBER::BATTERY_BOX_EXTENSION_MINIMUM;
            std::cout << "Climber: constrained extension to: "
                    << extension.to<double>() << "m (battery box constraint)\n";
            result = true;
        }
    }

    // Handle the wall constraint
    units::meter_t minimum;
    /* minimum = k1 - ((k2 - v1) / v2)
     * k1: ARM_BACKWARD_LENGTH
     * k2: FRAME_BACK_OFFSET + FRAME_EXTENSION_MAXIMUM - SAFETY_MARGIN
     * v1: ARM_OFFSET + ARM_WIDTH * cosine(angle)
     * v2: sine(angle)
     */
    // k2
    minimum = (CONSTANTS::CLIMBER::FRAME_BACK_OFFSET
            + CONSTANTS::CLIMBER::FRAME_EXTENSION_MAXIMUM
            - CONSTANTS::CLIMBER::SAFETY_MARGIN);
    // minus v1
    minimum -= (CONSTANTS::CLIMBER::ARM_OFFSET + CONSTANTS::CLIMBER::ARM_WIDTH)
            * units::math::cos(angle);
    // divided by v2
    minimum /= units::math::sin(angle);
    // k1 minus
    minimum = CONSTANTS::CLIMBER::ARM_BACKWARD_LENGTH - minimum;
    if(extension < minimum) {
        extension = minimum;
        std::cout << "Climber: constrained extension to: "
                << extension.to<double>() << "m (wall constraint)\n";
        result = true;
    }

    // Handle the ceiling constraint
    units::meter_t maximum;
    /* maximum = ((k1 - v1) / v2) - k2
     * k1: HEIGHT_MAXIMUM - HEIGHT_OFFSET
     * v1: ARM_OFFSET + HOOK_WIDTH * sine(angle)
     * v2: cosine(angle)
     * k2: ARM_FORWARD_LENGTH
     */
    // k1
    maximum = (CONSTANTS::CLIMBER::HEIGHT_MAXIMUM
            - CONSTANTS::CLIMBER::HEIGHT_OFFSET);
    // minus v1
    maximum -= (CONSTANTS::CLIMBER::ARM_OFFSET + CONSTANTS::CLIMBER::HOOK_WIDTH)
            * units::math::sin(angle);
    // divided by v2
    maximum /= units::math::cos(angle);
    // minus k2
    maximum -= CONSTANTS::CLIMBER::ARM_FORWARD_LENGTH;
    if(extension > maximum) {
        extension = maximum;
        std::cout << "Climber: constrained extension to: "
                << extension.to<double>() << "m (ceiling constraint)\n";
        result = true;
    }

    extend->Set(extension);
    rotate->Set(angle);
    return result;
}

void Climber::Periodic() {
    frc::SmartDashboard::PutNumber("Climber extension setpoint",
            extend->Get().to<double>());
    frc::SmartDashboard::PutNumber("Climber angle setpoint",
            rotate->Get().to<double>());

    frc::SmartDashboard::PutNumber("Climber extension",
            extend->GetActual().to<double>());
    frc::SmartDashboard::PutNumber("Climber angle",
            rotate->GetActual().to<double>());
}
