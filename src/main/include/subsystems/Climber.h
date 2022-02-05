#pragma once
#include "frc2/command/SubsystemBase.h"
#include "subsystems/climber/ClimberExtend.h"
#include "subsystems/climber/ClimberRotate.h"


namespace CONSTANTS::CLIMBER {
    // Hard limits
    constexpr units::meter_t
            EXTENSION_MAXIMUM = 3_ft + 1_in,
            EXTENSION_MINIMUM = 0_in;
    constexpr units::degree_t
            ANGLE_MAXIMUM = 90_deg,
            ANGLE_MINIMUM = 0_deg;

    /*
     * Battery box constraint
     *
     * The arm must be atleast EXTENSION_MINIMUM meters from the bottom
     * of travel when between ANGLE_MINIMUM and ANGLE_MAXIMUM.
     */
    constexpr units::meter_t
            BATTERY_BOX_EXTENSION_MINIMUM = 8_in;
    constexpr units::degree_t
            BATTERY_BOX_ANGLE_MAXIMUM = 15_deg,
            BATTERY_BOX_ANGLE_MINIMUM = 5_deg;

    /*
     * Wall constraint
     *
     * The climber must be not extend past 16 inches from the end of the frame.
     */
    constexpr units::meter_t
            // Offset from the back of the frame to the rotation point
            FRAME_BACK_OFFSET = 26.5_in,
            // Max extension past the frame (G107 & R105)
            FRAME_EXTENSION_MAXIMUM = 16_in,
            // Safety margin from the limits
            SAFETY_MARGIN = 6_in,
            // Offset of arm from rotation point
            ARM_OFFSET = 2.5_in,
            // Width of the arn
            ARM_WIDTH = 1_in,
            // Length from the rotation point to the bottom end of the arm
            ARM_BACKWARD_LENGTH = 35_in;

    /*
     * Ceiling constraint
     *
     * The climber must be not extend past 5 foot 6 inches from the floor.
     */
    constexpr units::meter_t
            // Max height (G106)
            HEIGHT_MAXIMUM = 5_ft + 6_in,
            // Offset from the floor to the rotation point
            HEIGHT_OFFSET = 3_ft + 4_in,
            // Width of the arm plus hook
            HOOK_WIDTH = 4_in,
            // Length from the rotation point to the top end of the arm
            ARM_FORWARD_LENGTH = 11_in;
}

class Climber : public frc2::SubsystemBase {
  private:
    ClimberExtend *extend;
    ClimberRotate *rotate;

  public:
    Climber();
    ~Climber();

    units::meter_t GetExtension();
    bool SetExtension(units::meter_t extension);

    units::degree_t GetAngle();
    bool SetAngle(units::degree_t angle);

    bool Set(units::meter_t extension, units::degree_t angle);
};
