// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include "TankDrive.h"
#include <frc/GenericHID.h>
using namespace OIConstants;
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
  TankDrive m_drive;
  frc::GenericHID m_driverController{kDriverControllerPort};
};
