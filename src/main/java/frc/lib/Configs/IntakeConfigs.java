package frc.lib.Configs;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class IntakeConfigs {
    public static final class IntakeConfig {
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
        
        static{
            intakeConfig
                .idleMode(IdleMode.kCoast)
                .secondaryCurrentLimit(20)
                .voltageCompensation(10);
            
            intakeConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        }
    }
}
