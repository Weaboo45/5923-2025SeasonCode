package frc.robot.subsystems.PIDSubsystems;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax rightArmMotor;
  private final SparkMax leftArmMotor;

  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ArmConstants.kEncoderPort);
  private final RelativeEncoder rightArmEncoder, leftArmEncoder;

  private final SparkClosedLoopController rightArmController;


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

    rightArmController.setP(Constants.ArmConstants.kP);
    rightArmController.setI(Constants.ArmConstants.kI);
    rightArmController.setD(Constants.ArmConstants.kD);
    rightArmController.setFF(Constants.ArmConstants.kF); //test kFF .5 doubled output

    // Setpoint Val
    SmartDashboard.putNumber("Arm Setpoint", getSetpoint());
  }

  private void configRightArmMotor() {
    rightArmMotor.restoreFactoryDefaults();
    rightArmMotor.setSmartCurrentLimit(15);
    rightArmMotor.setInverted(true);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.enableVoltageCompensation(12); 
    rightArmMotor.burnFlash();
    rightArmController.setOutputRange(-.5, .5);
  }

  private void configLeftArmMotor() {
    leftArmMotor.restoreFactoryDefaults();
    leftArmMotor.setSmartCurrentLimit(15);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.enableVoltageCompensation(12);
    leftArmMotor.burnFlash();
  }

  private void resetArmEncoders(){
    rightArmEncoder.setPosition(getRevEncoder()); //try subtracting
    leftArmEncoder.setPosition(getRevEncoder());
  }

  @Override
  public void periodic() {
    resetArmEncoders();

    SmartDashboard.putNumber("Arm Error", getArmError());

    SmartDashboard.putNumber("Shooter Angle", getRevEncoder());
    SmartDashboard.putNumber("Arm Angle", getArmAngle());

    //Changable PID vals from dashboard
    /*Left for example purposes */
    double newSet = SmartDashboard.getNumber("Arm Setpoint", setpoint);
      if(newSet != getSetpoint()){  setSetpoint(newSet); }

    rightArmController.setReference(getSetpoint(), ControlType.kPosition);

    Logger.recordOutput("Shooter Angle", getRevEncoder());
    Logger.recordOutput("Arm Angle", getArmAngle());
  }

  /** Arm Angle in degrees */
  private double getArmAngle(){
    double avg = rightArmEncoder.getPosition();
    return avg;
  }

  /** Shooter Angle in degrees */ //shooter is ~60 degrees offset from arm so when arm angle == 0, shooter angle == 60 in CAD
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
