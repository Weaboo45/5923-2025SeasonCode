package frc.lib.Configs;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class ElevatorConfigs {
    //SparkMax ElevatorMotor = new SparkMax(id???, MotorType.kBrushless);
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    static {
        elevatorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        elevatorConfig.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000);
        elevatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.4, 0.0, 0.0);
        
    }
}
