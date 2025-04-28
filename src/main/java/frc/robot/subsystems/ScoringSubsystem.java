package frc.robot.subsystems;

import frc.lib.Configs.IntakeConfigs.*;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {
    
    public SparkFlex intakeMotor;
    public SparkMax climbMotor, armMotor;

    public RelativeEncoder armEncoder;

    public DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(0);

    public Servo climbServo;

    public ScoringSubsystem(){

        climbServo = new Servo(0);

        intakeMotor = new SparkFlex(Constants.intakeID, MotorType.kBrushless);
        intakeMotor.configure(IntakeConfig.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climbMotor = new SparkMax(Constants.climbID, MotorType.kBrushless);
        climbMotor.configure(ClimbConfig.climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armMotor = new SparkMax(Constants.armID, MotorType.kBrushless);
        armMotor.configure(ArmConfig.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armEncoder = armMotor.getEncoder();

        resetArmEncoder();
    }

    @Override
    public void periodic(){
        resetArmEncoder();

        SmartDashboard.putNumber("Arm Angle", getArmAngle().getDegrees());
        //SmartDashboard.putNumber("Arm Rotations", armEncoder.getPosition());
    }

    /**outtakes coral .25 */
    public void coralOut(){
        intakeMotor.set(.5);
    }

    /**outtakes algae -.25 */
    public void algaeOut(){
        intakeMotor.set(-.5);
    }

    /**intakes algae .15 */
    public void algaeIn(){
        intakeMotor.set(.15);
    }

    public void intakeOff(){
        intakeMotor.set(0);
    }

    public void climbStop(){
        climbMotor.set(0);
        climbServo.setAngle(30);
    }

    public void climbDown(){
        climbMotor.set(.5);
        climbServo.setAngle(0);
    }

    public void climbUp(){
        climbMotor.set(-3);
        climbServo.setAngle(30);
    }

    public void armDown(){
        armMotor.set(-.35);
    }

    public void armUp(){
        armMotor.set(.35);
    }

    public void armStop(){
        armMotor.set(0);
    }

    /* 
    public boolean belowMaxAngle() {
        if(getArmAngle().getDegrees() < maxAngle){
            return true;
        } else {
            return false;
        }
    }

    public boolean aboveMinAngle() {
        if(getArmAngle().getDegrees() > minAngle){
            return true;
        } else {
            return false;
        }
    }
        */

    private void resetArmEncoder(){
        armEncoder.setPosition(getArmAngle().getRotations()); //try subtracting
    }

    /** Arm Angle in degrees */
    public Rotation2d getArmAngle(){
        Rotation2d armAngle = Rotation2d.fromRotations(absoluteEncoder.get() - (Constants.ArmConstants.encoderOffset / 360));
        return armAngle;
    }
}
