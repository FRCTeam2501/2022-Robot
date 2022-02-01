#pragma once
#include "wpi/sendable/Sendable.h"
#include "wpi/sendable/SendableHelper.h"
#include "rev/CANSparkMax.h"
#include "units/angle.h"


class ClimberRotate : public wpi::Sendable,
                      public wpi::SendableHelper<ClimberRotate> {
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
