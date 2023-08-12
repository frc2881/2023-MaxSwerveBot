// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static final class Controllers {
    public static final int kDriverControllerPort = 0; 
    public static final int kManipulatorControllerPort = 1; 
    public static final double kDeadband = 0.1; 
  }

  public static final class Drive {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 3 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.75);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.25);
    // Distance between front and rear wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 4.285 - (Math.PI/2); 
    public static final double kFrontRightChassisAngularOffset = 5.977; 
    public static final double kRearLeftChassisAngularOffset =  3.733 - Math.PI; 
    public static final double kRearRightChassisAngularOffset = 5.443 + (Math.PI/2); 

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 15;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 16;
    public static final int kRearRightDrivingCanId = 9;

    public static final int kFrontLeftTurningCanId = 14;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 17;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;

    public static final double kMaxRoll = 14.0;
    public static final double kMinRoll = 13.0;

    public static final double precisionMultiplier = 0.7;
  }
    
  public static final class SwerveModule {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kNeoMotorFreeSpeedRpm = 5676 * 0.9;

    public static final double kDrivingMotorFreeSpeedRps = kNeoMotorFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762; // 3 : 0.0762; // 3.75 : 0.09525; //3.8 : 0.09652; // 4 : 0.1016; //Units.inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15.0);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kSteeringMotorReduction = 150.0 / 7.0;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI);// / kSteeringMotorReduction; // radians
    public static final double kTurningEncoderVelocityFactor = ((2 * Math.PI) / kSteeringMotorReduction) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians/ -math.pi
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians/ -math.pi

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }


  public static final class Autonomous {
    
    public static final double kMoveMaxVelocity = 1.5; // 3.0

    public static final double kMoveMaxAccel = 1.5; // 3.0

    public static final double kMoveToBalanceMaxVelocity = 2.0; // 2.0

    public static final double kMoveToBalanceMaxAccel = 3.0; // 3.0

    public static final double kBalanceMaxVelocity = 1.5;

    public static final double kBalanceMaxAccel = 1.5;

    public static final double kPickupMaxVelocity = 1.0;

    public static final double kPickupMaxAccel = 1.0;
  }

}
