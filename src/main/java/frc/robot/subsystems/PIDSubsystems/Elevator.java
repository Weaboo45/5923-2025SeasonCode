package frc.robot.subsystems.PIDSubsystems;

import frc.robot.Constants.ShooterConstants;

import org.ejml.ops.FSemiRing;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  //shooter motors
  private SparkMax elevatorMotor;

  //relative encoders
  private RelativeEncoder elevatorEncoder;

  private Rev2mDistanceSensor distanceFinder;

  //shooter PID controllers
  private SparkClosedLoopController elevatorController;

  //PID variables
  public double maxDist = 1 , distSetpoint = 0;

  public Elevator(){
    //motors
    elevatorMotor = new SparkMax(ShooterConstants.topShooterMotorID, MotorType.kBrushless);
        
    //encoders
    elevatorEncoder = elevatorMotor.getEncoder();

    //PID controller
    elevatorController = elevatorMotor.getClosedLoopController();

    SmartDashboard.putNumber("Setpoint (m)", distSetpoint);

    resetEncoders();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Setpoint MPS", getSetpoint());
    //Kept here for example
    //read setpoint
    //double newSetpoint = SmartDashboard.getNumber("Setpoint", 0);
    //if((newSetpoint != setpoint)&& newSetpoint <= maxRPM) { setpoint = newSetpoint;}

    /* Kept here for example 
    double newMPS = SmartDashboard.getNumber("MPS Setpoint", 0);
    if((newMPS != mpsSetpoint)&& newMPS <= maxMPS){
      mpsSetpoint = newMPS;
      setpoint= mpsToRPM(mpsSetpoint);
    }
    */

    //elevatorController.setReference(getSetpoint(), ControlType.kPosition);

  }

  public void resetEncoders()
  {
    elevatorEncoder.setPosition(0);
  }

  public double getInches()
  {
    double inches = Units.metersToInches(distanceFinder.get());
    return inches;
  }

  public inchesToRotations(double inches)
  {
    //(29/32) in diameter
    
  }
  
  public double getDistance()
  {
    double inches = Units.inchesToRotations(90);
    return inches;
  }
  

  /** Setpoint in meters / second */
  public void setSetpoint(double newMPS){
  }

  public double getSetpoint(){
    return distSetpoint;
  }
}
