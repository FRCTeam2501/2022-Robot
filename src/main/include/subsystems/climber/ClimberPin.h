#pragma once
#include "frc/Solenoid.h"
#include "Constants.h"


class ClimberPin {
  private:
    frc::Solenoid *pin;

  public:
    ClimberPin();

    void Open();
    void Close();

    bool Get();
};
