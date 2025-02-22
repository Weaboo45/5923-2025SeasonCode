package frc.robot.subsystems.PIDSubsystems;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.lib.Configs.ArmConfigs;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax rightArmMotor;
  private final SparkMax leftArmMotor;

  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ArmConstants.kEncoderPort);
  private final RelativeEncoder rightArmEncoder, leftArmEncoder;

  private final SparkClosedLoopController rightArmController;

  private SparkMaxConfig armPIDConfig = new SparkMaxConfig();
  public double setpoint = 45;
  public double kP = 0, kI = 0, kD = 0, kF = 0;
  
  public ArmSubsystem() {
    rightArmMotor = new SparkMax(ArmConstants.rightArmMotorID, MotorType.kBrushless);
    leftArmMotor = new SparkMax(ArmConstants.leftArmMotorID, MotorType.kBrushless);

    rightArmEncoder = rightArmMotor.getEncoder();
    leftArmEncoder = leftArmMotor.getEncoder();

    rightArmController = rightArmMotor.getClosedLoopController();

    configLeftArmMotor();
    configRightArmMotor();
    resetArmEncoders();

    //leftArmMotor.follow(rightArmMotor, true);

    // PID values
    //Left for example purposes 
    SmartDashboard.putNumber("Arm P", kP);
    SmartDashboard.putNumber("Arm I", kI);
    SmartDashboard.putNumber("Arm D", kD);
    SmartDashboard.putNumber("Arm FF", kF);

    // Setpoint Val
    SmartDashboard.putNumber("Arm Setpoint", getSetpoint());
  }

  private void configRightArmMotor() {
    rightArmMotor.configure(ArmConfigs.rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configLeftArmMotor() {
    leftArmMotor.configure(ArmConfigs.leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void resetArmEncoders(){
    rightArmEncoder.setPosition(getRevEncoder()); //try subtracting
    leftArmEncoder.setPosition(getRevEncoder());
  }

  @Override
  public void periodic() {
    resetArmEncoders();

    SmartDashboard.putNumber("Arm Error", getArmError());

    SmartDashboard.putNumber("Arm Angle", getArmAngle());

    /* Kept here for example */
    double newkP = SmartDashboard.getNumber("Elevator P", 0);
    if(newkP != kP){  kP = newkP;  }

    double newkI = SmartDashboard.getNumber("Elevator I", 0);
    if(newkI != kI){  kI = newkI;  }

    double newkD = SmartDashboard.getNumber("Elevator D", 0);
    if(newkD != kD){  kD = newkD;  }
    

    armPIDConfig.closedLoop.pid(kP, kI, kD);

    rightArmMotor.configure(armPIDConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Changable PID vals from dashboard
    /*Left for example purposes */
    double newSet = SmartDashboard.getNumber("Arm Setpoint", setpoint);
      if(newSet != getSetpoint()){  setSetpoint(newSet); }

    rightArmController.setReference(getSetpoint(), ControlType.kPosition);

    Logger.recordOutput("Rev Encoder", getRevEncoder());
    Logger.recordOutput("Arm Angle", getArmAngle());
  }

  /** Arm Angle in degrees */
  private double getArmAngle(){
    return rightArmEncoder.getPosition() * 360; // may need to be tweaked
  }

  /** Rev Encoder in degrees */
  public double getRevEncoder(){
    return absoluteEncoder.get() * 360; // look at for change
  }

  //Methods for PID
  public double getArmError(){
    return getSetpoint() - rightArmEncoder.getPosition();
  }

  /**The new arm set point in degrees */
  public void setSetpoint(double newPoint){
    setpoint = newPoint;
  }

  public double getSetpoint(){
    return setpoint;
  }
}
