#pragma once


namespace JOYSTICK {
	//Standard Logitech joystick button and axis numbers
	namespace BUTTON {
		constexpr uint32_t
			TRIGGER = 1,
			THUMB = 2,
			BUTTON_3 = 3,
			BUTTON_4 = 4,
			BUTTON_5 = 5,
			BUTTON_6 = 6,
			BUTTON_7 = 7,
			BUTTON_8 = 8,
			BUTTON_9 = 9,
			BUTTON_10 = 10,
			BUTTON_11 = 11,
			BUTTON_12 = 12;
	}
	namespace AXIS {
		constexpr uint32_t
			X = 0,
			Y = 1,
			R = 2,
			Z = 3;
	}
	namespace POV {
		//Logitech controller POV degrees
		constexpr uint32_t
			TOP = 0,
			TOP_RIGHT = 45,
			RIGHT = 90,
			BOTTOM_RIGHT = 135,
			BOTTOM = 180,
			BOTTOM_LEFT = 225,
			LEFT = 270,
			TOP_LEFT = 315;
	}
}