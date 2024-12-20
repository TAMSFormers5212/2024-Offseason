// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>

#include "RobotContainer.h"
#include "Constants.h"
#include "TankDrive.h"

using namespace std;
using namespace frc2;
using namespace OIConstants;
using namespace pathplanner;

RobotContainer::RobotContainer() : m_intake(OIConstants::kIntakeMotor), m_shooter(OIConstants::kShooterMotor) {
  ConfigureBindings();

  // m_chooser.SetDefaultOption(Autos::LINE, m_lineTest.get());
  // m_chooser.AddOption(Autos::LINE, m_lineTest.get());

  m_drive.SetDefaultCommand(RunCommand(
    [this] {
      double leftY = -m_driverController.GetRawAxis(Controller::leftYAxis);
      double leftX = m_driverController.GetRawAxis(Controller::leftXAxis);
      if (m_driverController.GetRawButtonPressed(Controller::A))
      {
        leftY = 0;
        leftX = 0;
      }

      m_drive.m_drive.ArcadeDrive(leftY, leftX);
    },
    { &m_drive }
  ));

  m_shooter.SetDefaultCommand(RunCommand(
    [this] {
      // if (m_driverController.GetRawAxis(Controller::rightTrigger) > 0.001) {
      //   m_shooter.setSpeed(m_driverController.GetRawAxis(Controller::rightTrigger) * 100);
      // }
      // else {
      //   m_shooter.setSpeed(0);
      // }

      // if (m_driverController.GetRawButton(Controller::B)) {
      //   m_shooter.setSpeed(100);
      // }
      // else {
      //   m_shooter.setSpeed(100);
      // }
      frc::SmartDashboard::PutNumber("leftShooterSpeed", m_shooter.getleftSpeed());
    },
    { &m_shooter }
  ));
}

void RobotContainer::ConfigureBindings() {

  JoystickButton leftYAxis(&m_driverController, Controller::leftYAxis);
  JoystickButton rightYAxis(&m_driverController, Controller::rightYAxis);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_lineTest.get();
}
