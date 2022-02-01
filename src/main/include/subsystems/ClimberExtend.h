#pragma once
#include "frc/motorcontrol/MotorControllerGroup.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/CANSparkMax.h"
#include "units/angle.h"


class ClimberExtend : public frc2::SubsystemBase {
  private:
    // Individual speed controllers
    rev::CANSparkMax *winch;

  public:
    ClimberExtend();
    ~ClimberExtend();

    void SetSpeed(double speed);
};
