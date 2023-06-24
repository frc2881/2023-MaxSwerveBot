// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drive;

public class DriveRobotCentric extends CommandBase {
  private final Drive m_drive;

  public DriveRobotCentric(Drive drive) {
    m_drive = drive;
  }

  @Override
  public void initialize() {
    m_drive.setDriveMode(Drive.SwerveDriveMode.ROBOT_CENTRIC);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_drive.setDriveMode(Drive.SwerveDriveMode.FIELD_CENTRIC);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
