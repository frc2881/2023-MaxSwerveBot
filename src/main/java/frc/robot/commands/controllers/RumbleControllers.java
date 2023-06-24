// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.commands.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RumbleControllers extends SequentialCommandGroup {

  public static enum RumblePattern {
    GOOD,
    BAD,
  }

  public RumbleControllers(
    XboxController driverController,
    XboxController manipulatorController,
    RumblePattern pattern
  ) {

    if (pattern == RumblePattern.GOOD) {
      addCommands(
        new InstantCommand(() -> {
          driverController.setRumble(RumbleType.kBothRumble, 1);
          manipulatorController.setRumble(RumbleType.kBothRumble, 1);
        }),
        new WaitCommand(1),
        new InstantCommand(() -> {
          driverController.setRumble(RumbleType.kBothRumble, 0);
          manipulatorController.setRumble(RumbleType.kBothRumble, 0);
        }) 
      );
   } else {
      addCommands(
        new InstantCommand(() -> {
          driverController.setRumble(RumbleType.kBothRumble, 1);
          manipulatorController.setRumble(RumbleType.kBothRumble, 1);
        }),
        new WaitCommand(0.25),
        new InstantCommand(() -> {
          driverController.setRumble(RumbleType.kBothRumble, 0);
          manipulatorController.setRumble(RumbleType.kBothRumble, 0);
        }),
        new WaitCommand(0.25),
        new InstantCommand(() -> {
          driverController.setRumble(RumbleType.kBothRumble, 1);
          manipulatorController.setRumble(RumbleType.kBothRumble, 1);
        }),
        new WaitCommand(0.25),
        new InstantCommand(() -> {
          driverController.setRumble(RumbleType.kBothRumble, 0);
          manipulatorController.setRumble(RumbleType.kBothRumble, 0);
        })
      );
   }
  }
}
