#pragma once
#include "frc/motorcontrol/MotorControllerGroup.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"
#include "units/angle.h"


class ClimberRotate : public frc2::SubsystemBase {
  private:
    // Individual speed controller
    rev::CANSparkMax *rotation;
    // State variables
    units::degree_t angle;

  public:
    ClimberRotate();
    ~ClimberRotate();

    units::degree_t GetAngle();
    void SetAngle(units::degree_t angle);
};