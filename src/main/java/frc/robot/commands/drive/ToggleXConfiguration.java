// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class ToggleXConfiguration extends CommandBase {
  private final Drive m_drive;

  public ToggleXConfiguration(Drive drive) {
    m_drive = drive;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.toggleX();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
