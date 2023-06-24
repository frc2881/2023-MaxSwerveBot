// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class ZeroHeadingToAng extends CommandBase {
  private final Drive m_drive;
  private double m_angle;

  public ZeroHeadingToAng(Drive drive, double angle) {
    m_drive = drive;
    m_angle = angle;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_drive.zeroHeadingToAng(m_angle);
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
