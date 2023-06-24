// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class SetXConfiguration extends CommandBase {
  private final Drive m_drive;
  /** Creates a new SetX. */
  public SetXConfiguration(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setXConfiguration();

    // if X is false and its not been pushed, do nothing
    // if u push the button, x goes to true and pushed is set to false
    // if u push the button again, x goes to false, pushed goes to true and setX ends

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
