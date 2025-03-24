/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.OperatorCommands.ControllerCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystems.SwerveSubsystem;


public class DriveSwerve extends Command {
  /*
   * Creates a new DriveMecanum.
   */

  private SwerveSubsystem drivetrain;
  private Supplier<Double>  y, x, z;
  private Supplier<Boolean> lowPower, highPower;//fieldTOrientated, resetGyro;
  boolean fieldDrive = true;
  double speedMult = 2;

  private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);//2
  private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);//2
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.0);//4

  public DriveSwerve(SwerveSubsystem drivetrain, Supplier<Double> xDirect, Supplier<Double> yDirect, 
  Supplier<Double> rotation, Supplier<Boolean> lowPower, Supplier<Boolean> highPower){ //, Supplier<Boolean> fieldTOrientated, Supplier<Boolean> resetGyro){ 
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.y = yDirect;
    this.x = xDirect;
    this.z = rotation;
    this.lowPower = lowPower;
    this.highPower = highPower;
    //this.resetGyro = resetGyro;
    //this.fieldTOrientated = fieldTOrientated; // toggle
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if(lowPower.get()){
      speedMult = 2; //.5
    }

    if(highPower.get()){
      speedMult = 4.8; //3
    }

    /*
    SmartDashboard.putBoolean("Field Drive", fieldDrive);
    if(fieldTOrientated.get()){
      fieldDrive = !fieldDrive;
    }
      */

    /* Get Values, Deadband */
    double yVal = yLimiter
        .calculate(MathUtil.applyDeadband(y.get(), Constants.SPEED_DEADBAND));
    double xVal = xLimiter
        .calculate(MathUtil.applyDeadband(x.get(), Constants.STRAFING_DEADBAND));
    double rotationVal = rotationLimiter
        .calculate(MathUtil.applyDeadband(z.get(), Constants.ROTATION_DEADBAND));


    drivetrain.drive( new Translation2d(xVal * speedMult, yVal * speedMult) ,rotationVal * (speedMult + 1) , fieldDrive);

    SmartDashboard.putNumber("Speed Mult", speedMult);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
