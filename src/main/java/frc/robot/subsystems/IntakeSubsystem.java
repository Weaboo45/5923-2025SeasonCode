package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;

    private static SparkMaxConfig intakeConfig = new SparkMaxConfig();

    public void SubsystemBase(){
        intakeMotor = new SparkMax(Constants.intakeID, MotorType.kBrushless);

        configMotor();
    }

    public void configMotor(){
        intakeConfig
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(10)
            .smartCurrentLimit(30);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void outTake(){
        intakeMotor.set(1);
    }

    public void intakeOff(){
        intakeMotor.set(0);
    }
}
