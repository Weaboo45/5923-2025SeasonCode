package frc.lib.Configs;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class IntakeConfigs {
    public static final class IntakeConfig {
        public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
        
        static{
            intakeConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20)
                .voltageCompensation(10);
        }
    }

    public static final class ClimbConfig {
        public static final SparkMaxConfig climbConfig = new SparkMaxConfig();

        static {
            climbConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12);
        }
    }

    public static final class ArmConfig {
        public static final SparkMaxConfig armConfig = new SparkMaxConfig();

        static{
            armConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(10);

            armConfig.encoder
                .positionConversionFactor(Constants.ArmConstants.ARM_MOTOR_PCONVERSION);
                //.velocityConversionFactor(1);

            armConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder) //kAbsoluteEncoder
                    .pid(0.0,0,0) //Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD
                    .outputRange(-180, 180);
        }
    }
}
