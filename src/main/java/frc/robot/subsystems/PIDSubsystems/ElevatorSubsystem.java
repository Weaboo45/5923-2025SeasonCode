package frc.robot.subsystems.PIDSubsystems;

import frc.lib.Configs.ElevatorConfigs;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  //shooter motors
  private SparkMax elevatorMotor;

  //relative encoders
  private RelativeEncoder elevatorEncoder;

  private Rev2mDistanceSensor distanceFinder;

  //shooter PID controllers
  private SparkClosedLoopController elevatorController;
  private SparkMaxConfig elevatorConfig = new SparkMaxConfig();

  //PID variables
  public double kP = 0, kI = 0, kD = 0;
  public double maxDist = 50 , setpoint = 10; // setpoint && maxDist is in inches

  public ElevatorSubsystem(){
    //motors
    elevatorMotor = new SparkMax(Constants.ElevatorConstants.elevatorMotorID, MotorType.kBrushless);
    elevatorMotor.configure(ElevatorConfigs.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    //encoders
    elevatorEncoder = elevatorMotor.getEncoder();

    //PID controller
    elevatorController = elevatorMotor.getClosedLoopController();

    distanceFinder.setDistanceUnits(Unit.kInches);

    SmartDashboard.putNumber("Elevator Setpoint", setpoint);

    SmartDashboard.putNumber("Elevator P", kP);
    SmartDashboard.putNumber("Elevator I", kI);
    SmartDashboard.putNumber("Elevator D", kD);

    resetEncoders();
  }


  @Override
  public void periodic() {
    //getError();
    //getSetpoint();
    SmartDashboard.putNumber("Current Setpoint (in)", getSetpoint());
    SmartDashboard.putNumber("Error amount", getError());

    SmartDashboard.putNumber("Elevator Encoder (in)", elevatorEncoder.getPosition() * Constants.ElevatorConstants.CapCirc);
    SmartDashboard.putNumber("Elevator Encoder (rot)", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Setpoint (rot)", getSetpointRotations());

    //Kept here for example
    //read setpoint
    double newSetpoint = SmartDashboard.getNumber("Elevator Setpoint", 10);
    if((newSetpoint != setpoint)&& newSetpoint <= maxDist) { setpoint = newSetpoint;}

    /* Kept here for example */
    double newkP = SmartDashboard.getNumber("Elevator P", 0);
    if(newkP != kP){  kP = newkP;  }

    double newkI = SmartDashboard.getNumber("Elevator I", 0);
    if(newkI != kI){  kI = newkI;  }

    double newkD = SmartDashboard.getNumber("Elevator D", 0);
    if(newkD != kD){  kD = newkD;  }
    

    elevatorConfig.closedLoop.pid(kP, kI, kD);

    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorController.setReference(getSetpointRotations(), ControlType.kPosition);
  }

  public void resetEncoders(){
    elevatorEncoder.setPosition(0);
  }

  /**returns setpoint in inches */
  public double getSetpoint(){
    return setpoint;
  }

  /** returns the setpoint value in rotations */
  public double getSetpointRotations(){
    return setpoint / Constants.ElevatorConstants.CapCirc;
  }

  /** Setpoint in inches*/
  public Command setSetpoint(double newSetpoint){
    return this.runOnce(()-> setpoint = newSetpoint);
  }

  public double getError(){
    return setpoint - distanceFinder.getRange();
  }
}
