// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveSubsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.RelativeEncoder;

import frc.lib.SwerveModuleConstants;
import frc.lib.Configs.DriveConfigs;
import frc.robot.Constants;

public class SwerveModules {
  public int moduleNumber;

  private final SparkMax driveMotor;
  private final SparkMax turnMotor;

  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  private CANcoderConfiguration configs = new CANcoderConfiguration();
  private CANcoder absoluteEncoder;

  private final SparkClosedLoopController drivePIDController;
  private final SparkClosedLoopController turnPIDController;

  private double m_chassisAngularOffset = 0;
  //private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  //private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      //Constants.driveKS, Constants.driveKV, Constants.driveKA);

  /** Creates a new SwerveModule. */
  public SwerveModules(int moduleNumber, SwerveModuleConstants moduleConstants) {  //int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed
    this.moduleNumber = moduleNumber;

    driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    turnMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);

    absoluteEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    driveEncoder = driveMotor.getEncoder();
    drivePIDController = driveMotor.getClosedLoopController();

    turnEncoder = turnMotor.getEncoder();
    turnPIDController = turnMotor.getClosedLoopController();

    driveMotor.configure(DriveConfigs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    
    turnMotor.configure(DriveConfigs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    lastAngle = getState().angle;
    resetToAbsolute();
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    //desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
    //desiredState = new SwerveModuleState(desiredState.speedMetersPerSecond, getState().angle);
    //setAngle(desiredState);

    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(turnEncoder.getPosition())); //new Rotation2d(turnEncoder.getPosition())  getState().angle  absoluteEncoder.getPosition().getValueAsDouble())

    setSpeed(correctedDesiredState, isOpenLoop);
    setAngle(correctedDesiredState);

    SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] State", getState().toString());
    //Logger.recordOutput("Drivetrain/Module " + driveMotor.getDeviceId() + " State", getState());

    //Logger.recordOutput("Drivetrain/Module Desired States", desiredState);
  }

  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.DRIVETRAIN_MAX_SPEED; //Max drivetrain speed
      driveMotor.set(percentOutput);
    } else {
      drivePIDController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity);//,
          //ClosedLoopSlot.kSlot0,
          //feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.DRIVETRAIN_MAX_SPEED * 0.01))
        ? lastAngle
        : desiredState.angle;

    turnPIDController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    turnEncoder.setPosition(absolutePosition);
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  private void configAngleEncoder() {
    //configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    //configs.MountPose.MagnetOffset = 0.26;
    //configs.MountPose.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    absoluteEncoder.getConfigurator().apply(configs);
    absoluteEncoder.getPosition().setUpdateFrequency(100);
    absoluteEncoder.getVelocity().setUpdateFrequency(100);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(absoluteEncoder.getPosition().getValueAsDouble());
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(turnEncoder.getPosition());
  }

  public SwerveModuleState getState(){
    double velocity = driveEncoder.getVelocity();
    return new SwerveModuleState(velocity, getAngle());  
  }  
}