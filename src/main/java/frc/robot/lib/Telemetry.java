// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

public final class Telemetry {
  private static boolean m_isAllTelemetryEnabled = false;

  /** Initializes default telemetry settings and topics with custom periodic updates */
  public static void start() {
    SmartDashboard.putBoolean("IsAllTelemetryEnabled", m_isAllTelemetryEnabled);

    Robot.addCustomPeriodic(Telemetry::updateRobotInfo, 1);
    Robot.addCustomPeriodic(Telemetry::updateMatchInfo, 0.3);
    Robot.addCustomPeriodic(Telemetry::updateFPGATimestamp, 2.0);
    Robot.addCustomPeriodic(Telemetry::updateTelemetrySetting, 3.0);
  }

  /** This periodic function supports providing the current robot mode and status to the driver station dashbaoard app */
  private static void updateRobotInfo() {
    Robot.Mode mode = Robot.Mode.DISABLED;
    if (RobotState.isAutonomous()) { mode = Robot.Mode.AUTO; }
    if (RobotState.isTeleop()) { mode = Robot.Mode.TELEOP; }
    if (RobotState.isTest()) { mode = Robot.Mode.TEST; }
    SmartDashboard.putString("Robot/Mode", mode.toString());

    Robot.Status status = Robot.Status.DISABLED;
    if (RobotState.isEnabled()) { status = Robot.Status.ENABLED; }
    if (RobotState.isEStopped()) { status = Robot.Status.ESTOPPED; }
    SmartDashboard.putString("Robot/Status", status.toString());

    SmartDashboard.putNumber("Robot/BatteryVoltage", RobotController.getBatteryVoltage());
  }

  /** This periodic function supports providing the current match time to the driver station dashboard app during competition mode. */
  private static void updateMatchInfo() {
    SmartDashboard.putNumber("Timing/MatchTime", Math.floor(Timer.getMatchTime()));
  }

  /** This periodic function supports adding FPGA timestamps to entries when using the NetworkTables v3 protocol. */
  private static void updateFPGATimestamp() {
    SmartDashboard.putNumber("Timing/FPGATimestamp", Timer.getFPGATimestamp());
  }

  /** This periodic function supports enabling/disabling the full stream of telemetry pushed to LiveWindow. */
  private static void updateTelemetrySetting() {
    if (Robot.isCompetitionMode()) {
      if (m_isAllTelemetryEnabled) {
        m_isAllTelemetryEnabled = false;
        LiveWindow.disableAllTelemetry();
        SmartDashboard.putBoolean("IsAllTelemetryEnabled", false);
      }
    } else {
      boolean isAllTelemetryEnabled = SmartDashboard.getBoolean("IsAllTelemetryEnabled", m_isAllTelemetryEnabled);
      if (m_isAllTelemetryEnabled != isAllTelemetryEnabled) {
        m_isAllTelemetryEnabled = isAllTelemetryEnabled;
        if (m_isAllTelemetryEnabled) {
          LiveWindow.enableAllTelemetry();
        } else {
          LiveWindow.disableAllTelemetry();
        }
      }      
    }   
  }
}
