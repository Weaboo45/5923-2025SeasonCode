/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import frc.lib.SwerveModuleConstants;

//import com.pathplanner.lib.config.PIDConstants;

//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /* 
  /* Module Specific Constants */
    /* Front Left Module - Module 0 *
    public static final class Mod0 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); // Rotation2d.fromRadians(-Math.PI / 2); //Rotation2d.fromDegrees(0)
      public static final boolean driveMotorInverted = false;
      public static final boolean angleMotorInverted = true;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveMotorInverted, angleMotorInverted);
    }

    /* Front Right Module - Module 1 *
    public static final class Mod1 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); //Rotation2d.fromRadians(0); //Rotation2d.fromDegrees(0)
      public static final boolean driveMotorInverted = false;
      public static final boolean angleMotorInverted = true;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveMotorInverted, angleMotorInverted);
    }

    /* Back Left Module - Module 2 *
    public static final class Mod2 {
      public static final int driveMotorID = 8; //6
      public static final int angleMotorID = 7; //5
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); // Rotation2d.fromRadians(Math.PI / 2); //Rotation2d.fromDegrees(0)
      public static final boolean driveMotorInverted = false;
      public static final boolean angleMotorInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveMotorInverted, angleMotorInverted);
    }

    /* Back Right Module - Module 3 *
    public static final class Mod3 {
      public static final int driveMotorID = 6; //8
      public static final int angleMotorID = 5; //7
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0); //Rotation2d.fromRadians(Math.PI); //Rotation2d.fromDegrees(0)
      public static final boolean driveMotorInverted = false;
      public static final boolean angleMotorInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveMotorInverted, angleMotorInverted);
    }

    public static final int pidgeonID = 16;
    */

    public static final int armID = 14;
    public static final int intakeID = 13; // temporary
    public static final int climbID = 12;

    // Amp limits
    public static int PEAK_LIMIT = 40;
    public static int ENABLE_LIMIT = 30;

    // MEASUREMENTS
        // Drivetrain measurements 
        public static double CENTER_TO_WHEEL_X = Units.inchesToMeters(22.75/2); // Length   28/2
        public static double CENTER_TO_WHEEL_Y = Units.inchesToMeters(22.75/2); // width    28/2
        public static double WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        //Swerve Kinematics
        //may need to be changed || we want the outake to be "forward"
        public static SwerveDriveKinematics DRIVE_KIN = new SwerveDriveKinematics(
            new Translation2d(-CENTER_TO_WHEEL_X, CENTER_TO_WHEEL_Y),     //mod 0  - , +  || front left
            new Translation2d(CENTER_TO_WHEEL_X, CENTER_TO_WHEEL_Y),      //mod 1  + , +  || front right
            new Translation2d(-CENTER_TO_WHEEL_X, -CENTER_TO_WHEEL_Y),    //mod 2  - , -  || back left
            new Translation2d(CENTER_TO_WHEEL_X, -CENTER_TO_WHEEL_Y));    //mod 3  + , -  || back right

    // Drivetrain deadbands
    public static double ROTATION_DEADBAND = .1;   //.25
    public static double STRAFING_DEADBAND = .1;  //.75
    public static double SPEED_DEADBAND = .1; //.3

    //Drivetrain maxes
    public static double DRIVETRAIN_MAX_SPEED = 4.8; // m/s
    public static double DRIVETRAIN_MAX_TURN_SPEED = Math.PI * 2; // rads/s

    public static final double kFreeSpeedRpm = 5820; //neo motor rmp free speed

    //Drive motor Conversion Factors
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;

    public static final double kFreeWheelSpeedRps = (kFreeSpeedRpm * WHEEL_CIRCUMFERENCE) / DRIVE_MOTOR_GEAR_RATIO;

    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_CIRCUMFERENCE / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = (360/TURN_MOTOR_GEAR_RATIO); //360 / 2 * Math.PI  //in radians  TURN_MOTOR_GEAR_RATIO

    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double TURN_MOTOR_VCONVERSION =  TURN_MOTOR_GEAR_RATIO / 60.0; // (2 * Math.PI) / 60.0 //in radians

    // PID CONSTANTS
        // Drivetrain PID needs tuning
        public static double DRIVE_P = 0.1; //may need tuning
        public static double DRIVE_I = 0.001;
        public static double DRIVE_D = 0.05;
        public static double DRIVE_FF = 1 / kFreeWheelSpeedRps;

        public static double ROTATE_P = .01; //.01
        public static double ROTATE_I = 0.00001; //0.00
        public static double ROTATE_D = 0.0005; //.0005
        public static double ROTATE_FF = 0.0;

    // Autonomous drivetrain PID
    public static double AUTON_KP = 0;
    public static double AUTON_KI = 0;
    public static double AUTON_KD = 0;
    public static double AUTON_DISTANCE_SETPOINT = 0; // feet 3

    //Odometry
    public static final boolean invertGyro = false;

    public static final double driveKS = 0.1;
    public static final double driveKV = 2.3;
    public static final double driveKA = 0.3;

    public static final Translation2d MODULE_OFFSET = new Translation2d(CENTER_TO_WHEEL_X, CENTER_TO_WHEEL_Y);

    public static final class ArmConstants {

      //Arm motor Conversion Factors
      public static final double RevGearRatio = .01;
      public static final double PullyGearRatio = .57142857142;
      public static final double ARM_MOTOR_GEAR_RATIO = RevGearRatio * PullyGearRatio; // needs to change
      public static final double ARM_DIAMETER = Units.inchesToMeters(36); //needs to change

      public static final double ARM_MOTOR_PCONVERSION = ARM_DIAMETER * Math.PI / ARM_MOTOR_GEAR_RATIO;
      public static final double ARM_MOTOR_VCONVERSION = ARM_MOTOR_PCONVERSION / 60;

      //PID values
      public static final double kP = 0.05; //needs tuning
      public static final double kI = 0.00; //needs tuning
      public static final double kD = 0.05; //needs tuning

      //Button Setpoints in degrees
      public static final double homePoint = 0;

      public static final double encoderOffset = 191; // 183
    }
}