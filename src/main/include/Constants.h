// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "units/angle.h"
#include "units/current.h"


namespace Constants {



constexpr int FrontRight = 2;
constexpr int FrontLeft = 1;
constexpr int RearRight = 3;
constexpr int RearLeft = 5;

namespace DRIVETRAIN {
    /*
		constexpr units::current::ampere_t
					CURRENT_LIMIT = 60_A,
					HARD_CURRENT_LIMIT = 100_A;
		constexpr units::dimensionless::scalar_t
					GEAR_RATIO = 11.0;
		constexpr units::meter_t
					WHEEL_DIAMETER = 8_in,
					TRACK_WIDTH = 22_in,
					WHEEL_CIR = (WHEEL_DIAMETER * 3.141592);
		constexpr auto
					TURN_TO_METER = WHEEL_CIR * GEAR_RATIO / units::angle::turn_t(1);
		constexpr double
			//	Constant scalars to the robot's drive speed.
            
					Y_SPEED = -1.0,
					RZ_SPEED = 0.6
                    */
	}
    
}

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
