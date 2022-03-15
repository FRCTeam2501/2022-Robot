
//I coppy pasted this from 2020 code. I dont think I ever used it

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Encoder.h>
//#include <frc/PWMVictorSPX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/Command.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/ParallelRaceGroup.h>

//#include "Utils/Constants.h"
//#include "Utils/ButtonMap.h"
//#include "Utils/ports.h"

#include "rev/CANSparkMax.h"
#include "rev/CANPIDController.h"
#include "rev/CANEncoder.h"

//#include "ctre/Phoenix.h"
