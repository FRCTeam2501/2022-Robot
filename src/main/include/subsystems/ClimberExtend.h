#pragma once
#include "rev/CANSparkMax.h"
#include "wpi/sendable/Sendable.h"
#include "wpi/sendable/SendableHelper.h"
#include "units/angle.h"


class ClimberExtend : public wpi::Sendable,
                      public wpi::SendableHelper<ClimberExtend> {
  private:
    // Individual speed controllers
    rev::CANSparkMax *winch;

  public:
    ClimberExtend();
    ~ClimberExtend();

    void SetSpeed(double speed);
};
