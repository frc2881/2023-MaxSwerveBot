// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.lib;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule implements Sendable {

  public static enum Location {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight
  }

  private final Location m_location;

  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkMaxPIDController m_drivingPIDController;
  private final SparkMaxPIDController m_turningPIDController;

  private double[] m_samples = new double[50];
  private int m_index = 0;
  private double m_sum = 0.0;
  private double m_chassisAngularOffset = 0;

  private final DoubleLogEntry m_logDrivingAppliedOutput;
  private final DoubleLogEntry m_logDrivingBusVoltage;
  private final DoubleLogEntry m_logDrivingOutputCurrent;
  private final DoubleLogEntry m_logDrivingVelocity;

  private final DoubleLogEntry m_logTurningAbsoluteEncoderPosition;
  private final DoubleLogEntry m_logTurningRelativeEncoderPosition;
  private final DoubleLogEntry m_logTurningAbsoluteEncoderAverage;

  private final DoubleLogEntry m_logTurningAppliedOutput;
  private final DoubleLogEntry m_logTurningBusVoltage;
  private final DoubleLogEntry m_logTurningOutputCurrent;


  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(Location location, int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_location = location;

    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // SDS Module is inverted relative to the MAXSwerve
    for (int i = 0; i < 10; i += 1) {
      m_drivingSparkMax.setInverted(true); 
      if (m_drivingSparkMax.getLastError() != REVLibError.kOk ) {
        Logger.log(m_location.toString() + " swerve module driving motor controller inversion error.");
      } else {
        break;
      }
    }

    for (int i = 0; i < 10; i += 1) {
      m_turningSparkMax.setInverted(true); 
      if (m_turningSparkMax.getLastError() != REVLibError.kOk ) {
        Logger.log(m_location.toString() + " swerve module turning motor controller inversion error.");
      } else {
        break;
      }
    }

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(Constants.SwerveModule.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(Constants.SwerveModule.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(Constants.SwerveModule.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(Constants.SwerveModule.kTurningEncoderVelocityFactor);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(Constants.SwerveModule.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(Constants.SwerveModule.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(Constants.SwerveModule.kDrivingP);
    m_drivingPIDController.setI(Constants.SwerveModule.kDrivingI);
    m_drivingPIDController.setD(Constants.SwerveModule.kDrivingD);
    m_drivingPIDController.setFF(Constants.SwerveModule.kDrivingFF);
    m_drivingPIDController.setOutputRange(Constants.SwerveModule.kDrivingMinOutput,
      Constants.SwerveModule.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(Constants.SwerveModule.kTurningP);
    m_turningPIDController.setI(Constants.SwerveModule.kTurningI);
    m_turningPIDController.setD(Constants.SwerveModule.kTurningD);
    m_turningPIDController.setFF(Constants.SwerveModule.kTurningFF);
    m_turningPIDController.setOutputRange(Constants.SwerveModule.kTurningMinOutput,
        Constants.SwerveModule.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(Constants.SwerveModule.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(Constants.SwerveModule.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(Constants.SwerveModule.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(Constants.SwerveModule.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset; 

    DataLog log = DataLogManager.getLog();

    m_logDrivingAppliedOutput = new DoubleLogEntry(log, "Drive/Swerve/" + m_location.toString() + "/Driving/AppliedOutput");
    m_logDrivingBusVoltage = new DoubleLogEntry(log, "Drive/Swerve/" + m_location.toString() + "/Driving/BusVoltage");
    m_logDrivingOutputCurrent = new DoubleLogEntry(log, "Drive/Swerve/" + m_location.toString() + "/Driving/OutputCurrent");
    m_logDrivingVelocity = new DoubleLogEntry(log, "Drive/Swerve/" + m_location.toString() + "/Driving/Velocity");

    m_logTurningAbsoluteEncoderPosition = new DoubleLogEntry(log, "Drive/Swerve/" + m_location.toString() + "/Turning/AbsoluteEncoderPosition");
    m_logTurningRelativeEncoderPosition = new DoubleLogEntry(log, "Drive/Swerve/"+ m_location.toString() + "/Turning/RelativeEncoderPosition");
    m_logTurningAbsoluteEncoderAverage = new DoubleLogEntry(log, "Drive/Swerve/" + m_location.toString() + "/Turning/AbsoluteEncoderAverage");

    m_logTurningAppliedOutput = new DoubleLogEntry(log, "Drive/Swerve/"+ m_location.toString() + "/Turning/AppliedOutput");
    m_logTurningBusVoltage = new DoubleLogEntry(log, "Drive/Swerve/" + m_location.toString() + "/Turning/BusVoltage");
    m_logTurningOutputCurrent = new DoubleLogEntry(log, "Drive/Swerve/" + m_location.toString() + "/Turning/OutputCurrent");
  }

  public void sample(){
    m_logTurningRelativeEncoderPosition.append(m_turningEncoder.getPosition());
    m_sum -= m_samples[m_index];
    m_samples[m_index] = m_turningEncoder.getPosition();
    m_logTurningAbsoluteEncoderPosition.append(m_samples[m_index]);
    m_sum += m_samples[m_index];
    m_logTurningAbsoluteEncoderAverage.append(m_sum / 50);
    m_index += 1;
    if (m_index == 50) {
      m_index = 0;
    }
    m_logDrivingBusVoltage.append(m_drivingSparkMax.getBusVoltage());
    m_logDrivingOutputCurrent.append(m_drivingSparkMax.getOutputCurrent());
    m_logDrivingAppliedOutput.append(m_drivingSparkMax.getAppliedOutput());
    m_logDrivingVelocity.append(m_drivingEncoder.getVelocity());

    m_logTurningBusVoltage.append(m_turningSparkMax.getBusVoltage());
    m_logTurningOutputCurrent.append(m_turningSparkMax.getOutputCurrent());
    m_logTurningAppliedOutput.append(m_turningSparkMax.getAppliedOutput());

    SmartDashboard.putNumber("Drive/Swerve/" + m_location.toString() + "/TurningAbsoluteEncoderAverage", m_sum / 50);
  }

  // public void resetTurningEncoder() {
  //   if (!m_valid) { return; }
  //   double initialAngle = (m_sum / 50.0) - m_resetOffset;
  //   m_desiredState.angle = new Rotation2d(initialAngle);
  //   //m_drivingEncoder.setPosition(0.0);
  //   m_turningEncoder.setPosition(initialAngle);
  // }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    // m_drivingPIDController.setP(SmartDashboard.getNumber("P", Constants.SwerveModule.kDrivingP)); ONLY FOR TESTING: CAUSES LOOP OVERRUNS
    // m_drivingPIDController.setD(SmartDashboard.getNumber("D", Constants.SwerveModule.kDrivingD)); ONLY FOR TESTING: CAUSES LOOP OVERRUNS
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0.0);
  }

  public double getSteeringAbsolutePosition(){
   return m_turningEncoder.getPosition();
  }

  public double getDrivingRelativePosition(){
    return m_drivingEncoder.getPosition();
  }

  public double getDrivingVelocity(){
    return m_drivingEncoder.getVelocity();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    String key = m_location.toString() + "/";
    builder.addDoubleProperty(key + "Steering/AbsolutePosition", this::getSteeringAbsolutePosition, null);

    builder.addDoubleProperty(key + "Driving/Velocity", this::getDrivingVelocity, null);
  }
}