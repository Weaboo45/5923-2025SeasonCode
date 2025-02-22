package frc.lib.Configs;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class ElevatorConfigs {
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    static {
        elevatorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .voltageCompensation(10);
        
        elevatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            //.pid(0.4, 0.0, 0.0);
        
    }
}
