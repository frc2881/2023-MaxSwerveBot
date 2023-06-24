// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class FollowTrajectory extends SequentialCommandGroup {
	public FollowTrajectory(PathPlannerTrajectory trajectory, boolean isFirstPath, Drive drive) {
		PIDController xPidController = new PIDController(0.01, 0, 0);
		PIDController yPidController = new PIDController(0.01, 0, 0);
		PIDController tPidController = new PIDController(2.5, 0, 0);
		addCommands(
			new InstantCommand(() -> {
				// Reset odometry for the first path you run during auto
				if (isFirstPath) {
					drive.resetPose(trajectory.getInitialHolonomicPose());
				}
			}),
			new PPSwerveControllerCommand(
				trajectory,
				drive::getPose,
				Constants.Drive.kDriveKinematics,
				// The PID controllers set to 0 work best since the swerve modules are already
				// being tuned in the Drive PID Controllers.
				xPidController,
				yPidController,
				tPidController,
				drive::setModuleStates,
				true,
				drive));
	}
}
