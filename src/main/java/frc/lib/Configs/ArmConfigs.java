package frc.lib.Configs;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public final class ArmConfigs {
    public static final SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig lefttMotorConfig = new SparkMaxConfig();
    public static SparkBaseConfig leftMotorConfig;
    
    static{
        rightMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .inverted(true)
            .voltageCompensation(10);

        rightMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-.5, .5);

        lefttMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .inverted(false)
            .voltageCompensation(10)
            .follow(Constants.ArmConstants.rightArmMotorID);
    }
    
}