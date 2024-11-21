// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/GenericHID.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "Constants.h"
#include "TankDrive.h"
#include "Intake.h"
#include "Shooter.h"

using namespace OIConstants;

class RobotContainer {
public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

private:
  void ConfigureBindings();

  frc2::CommandPtr m_lineTest = pathplanner::PathPlannerAuto(Autos::LINE_TEST).ToPtr();

  frc::SendableChooser<frc2::Command*> m_chooser;

  TankDrive m_drive;
  Intake m_intake;
  Shooter m_shooter;

  frc::GenericHID m_driverController{ kDriverControllerPort };
  frc::GenericHID m_operatorController{ kOperatorControllerPort };
};
