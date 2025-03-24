package frc.lib.Configs;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;


public final class DriveConfigs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            //double drivingFactor = Constants.WHEEL_DIAMETER * Math.PI
            //         / Constants.DRIVE_MOTOR_GEAR_RATIO;
            //double turningFactor = 2 * Math.PI; // radian conversion
            double drivingVelocityFeedForward = 1 / Constants.kFreeWheelSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .voltageCompensation(12)
                    .smartCurrentLimit(30);
            drivingConfig.encoder
                    .positionConversionFactor(Constants.DRIVE_MOTOR_PCONVERSION) // meters
                    .velocityConversionFactor(Constants.DRIVE_MOTOR_VCONVERSION); // meters per second drivingFactor / 60.0
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .voltageCompensation(12)
                    .smartCurrentLimit(20);
            turningConfig.encoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    //.inverted(false) // true?
                    .positionConversionFactor(Constants.TURN_MOTOR_PCONVERSION); // turningFactor radians Constants.TURN_MOTOR_PCONVERSION
                    //.velocityConversionFactor(Constants.TURN_MOTOR_PCONVERSION/60); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder) //kAbsoluteEncoder
                    .pid(Constants.ROTATE_P,
                    Constants.ROTATE_I,
                    Constants.ROTATE_D)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(-90, 90); // turningFactor
        }
    }
}