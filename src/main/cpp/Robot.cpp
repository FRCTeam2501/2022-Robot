#include "Robot.h"

#include <frc2/command/CommandScheduler.h>


void TylerBot::RobotInit() {

}

void TylerBot::RobotPeriodic() {
  	frc2::CommandScheduler::GetInstance().Run();
}

void TylerBot::DisabledInit() {

}

void TylerBot::DisabledPeriodic() {

}

void TylerBot::AutonomousInit() {
	autoCommand = container.GetAutonomousCommand();
	if (autoCommand != nullptr) {
		autoCommand->Schedule();
	}
}

void TylerBot::AutonomousPeriodic() {

}

void TylerBot::TeleopInit() {
	if (autoCommand != nullptr) {
		autoCommand->Cancel();
		autoCommand = nullptr;
	}
}

void TylerBot::TeleopPeriodic() {

}

void TylerBot::TestPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<TylerBot>();
}
#endif
