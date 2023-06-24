// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AutoBalance extends CommandBase {
  private final Drive m_drive;
  private double m_speed;
  private int m_repeat;

  private static enum State {
    LookForMax,
    LookForMin,
    LookForOver,
    WaitToStop,
    Done
  }

  private State m_state;
  
  public AutoBalance(Drive drive, Boolean isForward) {
    m_drive = drive;
    if(isForward == true) {
      m_speed = 1.0;
    } else {
      m_speed = -1.0;
    }

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_state = State.LookForMax;

    m_drive.drive(m_speed * 0.75, 0.0, 0.0);
    
  }

  @Override
  public void execute() {
   

    switch(m_state){
      case LookForMax:
      {
        double roll = Math.abs(m_drive.getRoll());
        if(roll >= Constants.Drive.kMaxRoll){
          m_state = State.LookForMin;
        }
        break;
      }

      case LookForMin:
      {
        double roll = Math.abs(m_drive.getRoll());
        if(roll < Constants.Drive.kMinRoll){
          m_drive.drive(0.0, 0.0, 0.0);
          m_state = State.LookForOver;
        }
        break;
      }
      case LookForOver:
      {
        if(m_speed > 0.0){
          m_drive.drive(-m_speed/2, 0.0, 0.0);
          if(DriverStation.getAlliance() == Alliance.Red){
            m_repeat = 14;
          }
          else {
            m_repeat = 10;
          }
          
          m_state = State.WaitToStop;
        }
        if(m_speed < 0.0){
          m_drive.drive(-m_speed/2, 0.0, 0.0);
          if(DriverStation.getAlliance() == Alliance.Red){
            m_repeat = 14;
          }
          else {
            m_repeat = 10;
          }

          m_state = State.WaitToStop;
        }
        break;
      }
      case WaitToStop:
      {
        m_repeat -= 1;
        if(m_repeat == 0){
          m_drive.drive(0.0, 0.0, 0.0);
          m_state = State.Done;
        }
        break;
      }
      case Done:
      {
        break;
      }
      

    }

  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    if(m_state == State.Done){
      return true;
    } else {
      return false;
    }
  }
}
