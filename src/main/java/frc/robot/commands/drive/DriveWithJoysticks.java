// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.StateMachine;
import frc.robot.subsystems.Drive;

public class DriveWithJoysticks extends CommandBase {
  private final Drive m_drive;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private final DoubleSupplier m_thetaSupplier;
  private final BooleanSupplier m_lockRotationSupplier;
  private final PIDController m_thetaController;
  private final StateMachine m_stateMachine;
  private enum States {
    WaitForHold,
    HoldRotation,
    LockRotation,
    NumStates
  }
  private int m_zeroCount;
  private double m_translationX;
  private double m_translationY;
  private double m_rotation;

  public DriveWithJoysticks(
    Drive drive,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier,
    DoubleSupplier thetaSupplier,
    BooleanSupplier lockRotationSupplier
  ) {
    m_drive = drive;

    m_translationXSupplier = translationXSupplier;
    m_translationYSupplier = translationYSupplier;
    m_rotationSupplier = rotationSupplier;
    m_thetaSupplier = thetaSupplier;
    m_lockRotationSupplier = lockRotationSupplier;

    m_thetaController = new PIDController(0.01, 0, 0);
    m_thetaController.enableContinuousInput(-180.0, 180.0);
    m_thetaController.setTolerance(0.5, 0.5);

    m_stateMachine = new StateMachine(States.NumStates.ordinal(),
                                      States.WaitForHold.ordinal());

    m_stateMachine.addState(States.WaitForHold.ordinal(), this::runWaitForHold,
                            this::switchToWaitForHold);
    m_stateMachine.addState(States.HoldRotation.ordinal(),
                            this::runHoldRotation, this::switchToHoldRotation);
    m_stateMachine.addState(States.LockRotation.ordinal(),
                            this::runLockRotation, this::switchToLockRotation);

    m_zeroCount = 0;

    addRequirements(drive);
  }

  /**
   * Computes the cardinal direction to which the rotation should be locked.
   *
   * @return The angle for the rotation lock, or {@code -1} if the lock angle
   *         could not be determined.
   */
  private double lockAngleGet() {
    double angle = m_thetaSupplier.getAsDouble();
    angle = MathUtil.inputModulus(angle, -180, 180);

    if((angle > -30.0) && (angle < 30.0)) {
      return(0.0);
    }
    
    if((angle > 60.0) && (angle < 120.0)) {
      return(90.0);
    }

    if((angle > 150.0) || (angle < -150.0)) {
      return(180.0);
    }

    if((angle > -120.0) && (angle < -60.0)) {
      return(-90.0);
    }

    return(-1.0);
  }

  /**
   * Handles switching to the state that waits until the rotation should be
   * held.
   */
  private void switchToWaitForHold() {
    // Reset the zero counter.
    m_zeroCount = 0;
  }

  /**
   * Handles running the state that waits until the rotation should be held.
   */
  private void runWaitForHold() {
    // See if the rotation lock is requested and there is a valid cardinal
    // angle.
    if((m_translationX != 0.0) && (m_translationY != 0.0) &&
       m_lockRotationSupplier.getAsBoolean() && (lockAngleGet() != -1)) {
      // Switch to the rotation lock state.
      m_stateMachine.switchTo(States.LockRotation.ordinal());
      return;
    }

    // See if the robot is being commanded to move but not rotate.
    if(((m_translationX != 0.0) || (m_translationY != 0.0)) &&
       (m_rotation == 0.0)) {
      // Increment the zero counter.
      m_zeroCount++;

      // Switch to the rotation hold state if there have been enough
      // consecutive iterations with the robot being commanded to move but not
      // rotate.
      if(m_zeroCount >= 1) {
        m_stateMachine.switchTo(States.HoldRotation.ordinal());
        return;
      }
    } else {
      // Reset the zero counter.
      m_zeroCount = 0;
    }
  }

  /**
   * Handles switching to the state that holds the rotation.
   */
  private void switchToHoldRotation() {
    // Reset the theta PID controller.
    m_thetaController.reset();

    // Set the theta PID controller setpoint to the current robot theta.
    m_thetaController.setSetpoint(m_thetaSupplier.getAsDouble());
  }

  /**
   * Runs the state that holds the rotation.
   */
  private void runHoldRotation() {
    // See if the robot is no longer being commanded to move, or it is being
    // commanded to rotate.
    if(((m_translationX == 0.0) && (m_translationY == 0.0)) ||
       (m_rotation != 0.0)) {
      // Switch to the state that waits until the rotation should be held.
      m_stateMachine.switchTo(States.WaitForHold.ordinal());
      return;
    }
    
    // See if the rotation lock is requested and there is a valid cardinal
    // angle.
    if(m_lockRotationSupplier.getAsBoolean() && (lockAngleGet() != -1)) {
      // Switch to the rotation lock state.
      m_stateMachine.switchTo(States.LockRotation.ordinal());
      return;
    }

    // The theta should be held, so calculate the next iteration of the PID
    // controller.
    m_rotation = m_thetaController.calculate(m_thetaSupplier.getAsDouble());

    // Zero the rotation command if the PID controller is at its setpoint.
    if(m_thetaController.atSetpoint()) {
      m_rotation = 0.0;
    }
  }

  /**
   * Handles switching to the state that locks the rotation to a cardinal
   * direction.
   */
  private void switchToLockRotation() {
    // Reset the theta PID controller.
    m_thetaController.reset();

    // Set the theta PID controller setpoint to the closest lock angle.
    m_thetaController.setSetpoint(lockAngleGet());
  }

  /**
   * Runs the state that locks the rotation to a cardinal direction.
   */
  private void runLockRotation() {
    // See if the robot is no longer being commanded to move.
    // if((m_translationX == 0.0) && (m_translationY == 0.0)) {
    //   // Switch to the WaitForHold state.
    //   m_stateMachine.switchTo(States.WaitForHold.ordinal());
    //   return;
    // }

    // See if the rotation lock has been ended.
    if(!m_lockRotationSupplier.getAsBoolean()) {
      m_stateMachine.switchTo(States.WaitForHold.ordinal());
      return;
    }

    // Calculate the next iteration of the PID controller.
    m_rotation = m_thetaController.calculate(m_thetaSupplier.getAsDouble());

    // Zero the rotation command if the PID controller is at its setpoint.
    if(m_thetaController.atSetpoint()) {
      m_rotation = 0.0;
    }
  }

  @Override
  public void initialize() {
    m_stateMachine.switchTo(States.WaitForHold.ordinal());
  }

  @Override
  public void execute() {
    m_translationX = m_translationXSupplier.getAsDouble();
    m_translationY = m_translationYSupplier.getAsDouble();
    m_rotation = m_rotationSupplier.getAsDouble();

    m_stateMachine.run();

    m_drive.drive(m_translationX * Constants.Drive.kMaxSpeedMetersPerSecond,
                  m_translationY * Constants.Drive.kMaxSpeedMetersPerSecond,
                  m_rotation * Constants.Drive.kMaxAngularSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
