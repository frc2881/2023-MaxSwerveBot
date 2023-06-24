// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.Move;
import frc.robot.commands.auto.MoveToBalance;
import frc.robot.commands.auto.TestAuto;
import frc.robot.commands.drive.DrivePrecision;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.ToggleXConfiguration;
import frc.robot.commands.drive.ZeroHeading;
import frc.robot.lib.Utils;
import frc.robot.subsystems.Drive;

public class RobotContainer {
  private final PowerDistribution m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  private Drive m_drive = new Drive();

  private final XboxController m_driverController = new XboxController(Constants.Controllers.kDriverControllerPort);

  private final SendableChooser<Command> m_autonomousChooser = new SendableChooser<Command>();
 
  public RobotContainer() {
    setupDrive(); 
    setupControllers();
    setupAuto();
  }

  private void setupDrive() {
    m_drive.setDefaultCommand(
      new DriveWithJoysticks(
        m_drive,
        () -> Utils.applyDeadband(-m_driverController.getLeftY(), Constants.Controllers.kDeadband),
        () -> Utils.applyDeadband(-m_driverController.getLeftX(), Constants.Controllers.kDeadband),
        () -> Utils.applyDeadband(-m_driverController.getRightX(), Constants.Controllers.kDeadband),
        () -> m_drive.getHeading(),
        m_driverController::getRightBumper
      )
    );
    //m_drive.resetSwerve();
  }

  private void setupControllers() {
    
    // DRIVER CONTROLLER =========================

    new Trigger(() -> Math.abs(m_driverController.getRightTriggerAxis()) > 0.9)
      .whileTrue(new DrivePrecision(m_drive));

    new Trigger(m_driverController::getBackButton)
      .onTrue(new ZeroHeading(m_drive));

    // new Trigger(m_driverController::getAButton)
    //   .onTrue(new AlignToNearestNode(m_drive, m_drive::getTrajectoryForNearestNode));

    new Trigger(m_driverController::getBButton)
      .whileTrue(new AutoBalance(m_drive, false));
    
    new Trigger(m_driverController::getXButton)
      .onTrue(new ToggleXConfiguration(m_drive));

  }

  public void setupAuto() {
    
    PathPlannerTrajectory move1Path = PathPlanner.loadPath("Move 1", Constants.Autonomous.kMoveMaxVelocity, Constants.Autonomous.kMoveMaxAccel);
    PathPlannerTrajectory move9Path = PathPlanner.loadPath("Move 9", Constants.Autonomous.kMoveMaxVelocity, Constants.Autonomous.kMoveMaxAccel);
    
    PathPlannerTrajectory moveToBalance5Path = PathPlanner.loadPath("Balance 5", Constants.Autonomous.kMoveToBalanceMaxVelocity, Constants.Autonomous.kMoveToBalanceMaxAccel);
    PathPlannerTrajectory moveToBalance6Path = PathPlanner.loadPath("Balance 6", Constants.Autonomous.kMoveToBalanceMaxVelocity, Constants.Autonomous.kMoveToBalanceMaxAccel);
    PathPlannerTrajectory balanceMidPath = PathPlanner.loadPath("Mid Balance", Constants.Autonomous.kBalanceMaxVelocity, Constants.Autonomous.kBalanceMaxAccel);

    PathPlannerTrajectory testPath = PathPlanner.loadPath("Test", 2.0, 3.0);

    m_autonomousChooser.setDefaultOption("None", null);

    
    // TESTING

    m_autonomousChooser.addOption("TEST: 1 - Move", 
      new Move(m_drive, move1Path));

    m_autonomousChooser.addOption("TEST: 6 - Balance",
      new MoveToBalance(m_drive, moveToBalance6Path, balanceMidPath, true));

    m_autonomousChooser.addOption("TEST: 5 - Balance",
      new MoveToBalance(m_drive, moveToBalance5Path, balanceMidPath, true));


    m_autonomousChooser.addOption("TEST: 9 - Move", 
      new Move(m_drive, move9Path));

    m_autonomousChooser.addOption("TEST: Test", 
      new TestAuto(m_drive, testPath));


    SmartDashboard.putData("Auto/Command", m_autonomousChooser);
  }

  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }

  public void resetRobot() {
      //m_drive.resetSwerve();
  }
}
