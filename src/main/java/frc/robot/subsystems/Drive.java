// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.NavX;
import frc.robot.lib.PhotonCameraWrapper;
import frc.robot.lib.SwerveModule;

public class Drive extends SubsystemBase {

  public static enum SwerveDriveMode {
    FIELD_CENTRIC, 
    ROBOT_CENTRIC
  }

  public static enum SwerveDriveSpeedMode {
    NORMAL,
    PRECISION
  }

  private final SwerveModule m_frontLeft = new SwerveModule(
    SwerveModule.Location.FrontLeft,
    Constants.Drive.kFrontLeftDrivingCanId,
    Constants.Drive.kFrontLeftTurningCanId,
    Constants.Drive.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
    SwerveModule.Location.FrontRight,
    Constants.Drive.kFrontRightDrivingCanId,
    Constants.Drive.kFrontRightTurningCanId,
    Constants.Drive.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
    SwerveModule.Location.RearLeft,
    Constants.Drive.kRearLeftDrivingCanId,
    Constants.Drive.kRearLeftTurningCanId,
    Constants.Drive.kRearLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
    SwerveModule.Location.RearRight,
    Constants.Drive.kRearRightDrivingCanId,
    Constants.Drive.kRearRightTurningCanId, 
    Constants.Drive.kRearRightChassisAngularOffset);

  private final NavX m_gyro = new NavX();
  
  private SwerveDriveMode m_swerveDriveMode = SwerveDriveMode.FIELD_CENTRIC;

  private SwerveDriveSpeedMode m_swerveDriveSpeedMode = SwerveDriveSpeedMode.NORMAL;

  private boolean m_isXConfiguration = false;

  public PhotonCameraWrapper m_frontPhotonCamera;
  public PhotonCameraWrapper m_backPhotonCamera;

  private final DoubleLogEntry m_logRoll;
  private final DoubleLogEntry m_logYaw;
  private final DoubleLogEntry m_logPitch;

  private final SwerveDrivePoseEstimator m_poseEstimator = 
    new SwerveDrivePoseEstimator(
      Constants.Drive.kDriveKinematics, 
      Rotation2d.fromDegrees(m_gyro.getAngle()), 
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()}, 
      new Pose2d());

  public Drive() {
    DataLog log = DataLogManager.getLog();
    m_logRoll = new DoubleLogEntry(log, "/Drive/Roll");
    m_logYaw = new DoubleLogEntry(log, "/Drive/Yaw");
    m_logPitch = new DoubleLogEntry(log, "/Drive/Pitch");
  }

  @Override
  public void periodic() {
    updatePose();
    updateTelemetry();
    sampleModules();
    logDrive();
  }
  
 
  public void updatePose() {
    m_poseEstimator.update(
      m_gyro.getRotation2d(), 
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    });
  }

  /**
   * Returns the current estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
      pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    // Adjust input based on   speed
    // xSpeed *= Constants.Drive.kMaxSpeedMetersPerSecond;
    // ySpeed *= Constants.Drive.kMaxSpeedMetersPerSecond;
    // rot *= Constants.Drive.kMaxAngularSpeed;

    if(m_swerveDriveSpeedMode == SwerveDriveSpeedMode.PRECISION){
      xSpeed *= Constants.Drive.precisionMultiplier;
      ySpeed *= Constants.Drive.precisionMultiplier;
      rot *= Constants.Drive.precisionMultiplier;
    }
    
    if(!m_isXConfiguration){
      var swerveModuleStates = Constants.Drive.kDriveKinematics.toSwerveModuleStates(
        (m_swerveDriveMode == SwerveDriveMode.FIELD_CENTRIC)
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle()))
          : new ChassisSpeeds(xSpeed, ySpeed, rot));

      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drive.kMaxSpeedMetersPerSecond);
      

      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    }
  }

  public void setDriveMode(SwerveDriveMode driveMode) {
    m_swerveDriveMode = driveMode;
    SmartDashboard.putString("Drive/Swerve/Mode", m_swerveDriveMode.toString());
  }

  public void setSpeedMode(SwerveDriveSpeedMode driveSpeedMode) {
    m_swerveDriveSpeedMode = driveSpeedMode;
    SmartDashboard.putString("Drive/Swerve/SpeedMode", m_swerveDriveSpeedMode.toString());
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setXConfiguration() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void toggleX() {
    // if u push the button, x goes to true and pushed is set to false
    // if u push the button again, x goes to false, pushed goes to true and setX ends
    if (!m_isXConfiguration) {
      setXConfiguration();
    }
    m_isXConfiguration = !m_isXConfiguration;
    SmartDashboard.putBoolean("Drive/Swerve/IsXConfiguration", m_isXConfiguration);
  }


  public SwerveModuleState[] convertToModuleStates(double xTranslation, double yTranslation, double rotation) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xTranslation, yTranslation, rotation);

    SwerveModuleState[] moduleStates = Constants.Drive.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    return moduleStates;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    if (!m_isXConfiguration) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Drive.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(desiredStates[0]);
      m_frontRight.setDesiredState(desiredStates[1]);
      m_rearLeft.setDesiredState(desiredStates[2]);
      m_rearRight.setDesiredState(desiredStates[3]);
    }
  }

  private void sampleModules(){
    m_frontLeft.sample();
    m_frontRight.sample();
    m_rearLeft.sample();
    m_rearRight.sample();
  }
   
  // public void resetSwerve() {
  //   m_frontLeft.resetTurningEncoder();
  //   m_frontRight.resetTurningEncoder();
  //   m_rearLeft.resetTurningEncoder();
  //   m_rearRight.resetTurningEncoder();
  // }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public void zeroHeadingToAng(double angle) {
    m_gyro.resetToAng(angle);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.Drive.kGyroReversed ? -1.0 : 1.0);
  }

  public double getRoll() {
    return m_gyro.getRoll();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    
    m_frontLeft.initSendable(builder);
    m_frontRight.initSendable(builder);
    m_rearLeft.initSendable(builder);
    m_rearRight.initSendable(builder);

    m_gyro.initSendable(builder);
    builder.addDoubleProperty("Heading", this::getHeading, null);
    builder.addDoubleProperty("TurnRate", this::getTurnRate, null);
  }

  private void updateTelemetry() {
    Pose2d pose = getPose();
    SmartDashboard.putNumberArray("Drive/Pose",  new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });
  }

  public void logDrive() {
    m_logRoll.append(m_gyro.getRoll());
    m_logYaw.append(m_gyro.getYaw());
    m_logPitch.append(m_gyro.getPitch());
  }
}
