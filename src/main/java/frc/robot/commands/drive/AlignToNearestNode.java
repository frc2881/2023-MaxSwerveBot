// Copyright (c) 2022 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AlignToNearestNode extends SequentialCommandGroup {

	public AlignToNearestNode(Drive drive, Supplier<PathPlannerTrajectory> trajectory) {
		PIDController xPidController = new PIDController(0.01, 0, 0);
		PIDController yPidController = new PIDController(0.01, 0, 0);
		PIDController tPidController = new PIDController(5, 0, 0);

		addCommands(
			new InstantCommand(
				() -> {
					SmartDashboard.putString("Drive/DynamicTrajectory", trajectory.get().toString());
				}
			),
			new PPSwerveControllerCommand(
				trajectory.get(),
				drive::getPose,
				Constants.Drive.kDriveKinematics,
				xPidController,
				yPidController,
				tPidController,
				drive::setModuleStates,
				false,
				drive
			)
		);
	}
}
