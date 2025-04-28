
 
package frc.robot.commands.Autos;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveSubsystems.SwerveSubsystem;

public class SimpleAuto extends Command {
    
    private ScoringSubsystem scoringSub;
    private SwerveSubsystem drivetrain;
    private int phase;
    private Timer timer = new Timer();

    public SimpleAuto(ScoringSubsystem scoringSub, SwerveSubsystem drivetrain){
        addRequirements(scoringSub, drivetrain);
        this.drivetrain = drivetrain;
        this.scoringSub = scoringSub;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    phase = 1;
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timedAutoSequence();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new Translation2d(0, 0) ,0 , true); //drivetrain.drive(new Translation2d(0, 0) ,0 , true);
  }

  private void timedAutoSequence() {
    switch (phase){

      case 1: //drive forward to reef
       if(timer.get() < 2.0){
        drivetrain.drive(new Translation2d(1, 0) ,0, false);
       } else {
        drivetrain.drive(new Translation2d(0, 0) , 0, false);
        //phase++;
       }
       break; //end of case 1

       /* 
       case 2:  //outtake coral
       if(timer.get() > 1.175 && timer.get() < 3) {
        if(timer.get() > 2.125){
          scoringSub.coralOut();
        } 
       } else {
        scoringSub.intakeOff();
        phase++;
       }
       break; //end of case 2
       */

      default:
      drivetrain.drive(new Translation2d(0, 0) ,0 , true);
      scoringSub.intakeOff();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


