#pragma once
#include "rev/CANSparkMax.h"
#include "units/length.h"


class ClimberExtend {
  private:
    // Individual speed controllers
    rev::CANSparkMax *winch;
    // State variables
    units::meter_t distance;

  public:
    ClimberExtend();
    ~ClimberExtend();

    void SetExtension(units::meter_t distance);
    units::meter_t GetExtension();
};
