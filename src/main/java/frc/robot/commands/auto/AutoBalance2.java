// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.


package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class AutoBalance2 extends CommandBase {
  private final Drive m_drive;

  private final double kPBalancing = 0.02;
  private final double kIBalancing = 0.0;
  private final double kDBalancing = 0.0;
  private final double balancingTolerance = 2.0;

  private double pidMaxSpeed = 0.75;

  private boolean m_reversed;

  private int m_increment = 0;

  PIDController pidController = new PIDController(kPBalancing, kIBalancing, kDBalancing);

  /** Creates a new AutoBalance2. */
  public AutoBalance2(Drive drive, boolean reversed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_reversed = reversed;
    pidController.setTolerance(balancingTolerance);

    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setModuleStates(
      m_drive.convertToModuleStates(
        MathUtil.clamp(
            (m_reversed?-1:1)*pidController.calculate(m_drive.getRoll(), 0.0), 
            -pidMaxSpeed, 
            pidMaxSpeed), 
        0.0, 
        0.0)
    );
    
    if(pidController.atSetpoint()){ 
      m_increment++; 
    }
    else { 
      m_increment = 0; 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_increment == 20 || DriverStation.getMatchTime() < 0.4){
      return true;
    } else {
      return false;
    }
    
  }
}
