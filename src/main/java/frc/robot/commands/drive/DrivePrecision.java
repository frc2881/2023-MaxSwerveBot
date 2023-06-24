// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.


package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DrivePrecision extends CommandBase {
  private final Drive m_drive;

  /** Creates a new DrivePrecision. */
  public DrivePrecision(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setSpeedMode(Drive.SwerveDriveSpeedMode.PRECISION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setSpeedMode(Drive.SwerveDriveSpeedMode.NORMAL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
