// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/button/Trigger.h>
#include "RobotContainer.h"
#include "Constants.h"
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>

#include <frc2/command/RunCommand.h>

#include "TankDrive.h"
using namespace std;
using namespace frc2;
using namespace OIConstants;
RobotContainer::RobotContainer() {
  
  ConfigureBindings();
  m_drive.SetDefaultCommand(RunCommand(
        [this] {
            //Use driverController.GetRawAxis to get inputs and make the motors move based off of those inputs
            //Left joystick should control left motors, right joystick should control right motors
        }, {&m_drive}

    ));
}

void RobotContainer::ConfigureBindings() {

    JoystickButton leftYAxis(&m_driverController, Controller::leftYAxis);
    JoystickButton rightYAxis(&m_driverController, Controller::rightYAxis);
    
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
