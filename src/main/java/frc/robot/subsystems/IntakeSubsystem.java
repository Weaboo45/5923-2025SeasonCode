package frc.robot.subsystems;

//import frc.lib.Configs.IntakeConfigs;
import frc.lib.Configs.IntakeConfigs.IntakeConfig;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.config.SparkMaxConfig;
//mport com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    public SparkMax intakeMotor;
    
    public IntakeSubsystem(){

        intakeMotor = new SparkMax(Constants.intakeID, MotorType.kBrushless);
    
        intakeMotor.configure(IntakeConfig.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("MotorSpeed", intakeMotor.get());
    }

    public void intakeOn(){
        intakeMotor.set(.5); // m/s
    }

    public void intakeOff(){
        intakeMotor.set(0);
    }
}
