#pragma once
#include "frc2/command/SubsystemBase.h"
#include "subsystems/climber/ClimberExtend.h"
#include "subsystems/climber/ClimberRotate.h"


class Climber : public frc2::SubsystemBase {
  private:
    ClimberExtend *extend;
    ClimberRotate *rotate;

  public:
    Climber();
    ~Climber();

    units::meter_t GetExtension();
    void SetExtension(units::meter_t distance);

    units::degree_t GetAngle();
    void SetAngle(units::degree_t angle);
};
