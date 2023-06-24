// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.ZeroHeadingToAng;
import frc.robot.subsystems.Drive;

public class Move extends SequentialCommandGroup {

  public Move(
    Drive drive,
    PathPlannerTrajectory trajectory) {
      addCommands(
      new ZeroHeadingToAng(drive, 180),
      new FollowTrajectory(trajectory, true, drive)
    );
  }
}
