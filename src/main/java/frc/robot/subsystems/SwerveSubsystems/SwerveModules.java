// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveSubsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.RelativeEncoder;

import frc.lib.SwerveModuleConstants;
import frc.lib.Configs.DriveConfigs;

public class SwerveModules {

  private final SparkMax driveMotor;
  private final SparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  //private final RelativeEncoder turnEncoder;

  private CANcoderConfiguration configs = new CANcoderConfiguration();
  private CANcoder absoluteEncoder;

  private final SparkClosedLoopController drivePIDController;
  private final SparkClosedLoopController turnPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModules(int moduleNumber, SwerveModuleConstants moduleConstants) {
    driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    turnMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);

    absoluteEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    driveEncoder = driveMotor.getEncoder();
    //turnEncoder = turnMotor.getEncoder();
    
    drivePIDController = driveMotor.getClosedLoopController();
    turnPIDController = turnMotor.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    driveMotor.configure(DriveConfigs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    turnMotor.configure(DriveConfigs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    //resetToAbsolute();
    m_chassisAngularOffset = 0;
    m_desiredState.angle = new Rotation2d(absoluteEncoder.getPosition().getValueAsDouble()); //new Rotation2d(turnEncoder.getPosition())
    driveEncoder.setPosition(0);
  }

   /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveEncoder.getVelocity(),
        new Rotation2d(absoluteEncoder.getPosition().getValueAsDouble() - m_chassisAngularOffset));
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
        driveEncoder.getPosition(),
        new Rotation2d(absoluteEncoder.getPosition().getValueAsDouble() - m_chassisAngularOffset));
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
    correctedDesiredState.optimize(new Rotation2d(absoluteEncoder.getPosition().getValueAsDouble())); //new Rotation2d(turnEncoder.getPosition)  getState().angle

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePIDController.setReference(correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    turnPIDController.setReference(desiredState.angle.getDegrees(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
    SmartDashboard.putNumber("Turn Setpoint", desiredState.angle.getDegrees());
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  private void configAngleEncoder() {
    //configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    //configs.MountPose.MagnetOffset = 0.26;
    //configs.MountPose.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    absoluteEncoder.getConfigurator().apply(configs);
    absoluteEncoder.getPosition().setUpdateFrequency(100);
    absoluteEncoder.getVelocity().setUpdateFrequency(100);
  }

  /* 
  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees();
    turnEncoder.setPosition(absolutePosition);
  }
    */

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(absoluteEncoder.getPosition().getValueAsDouble());
  }

}