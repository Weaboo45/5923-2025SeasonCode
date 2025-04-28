
/* 
package frc.robot.commands.Autos;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveSubsystems.SwerveDrivetrain;

public class SimpleAuto extends Command {
    
    private IntakeSubsystem intakeSub;
    private SwerveDrivetrain drivetrain;
    private int phase;
    private Timer timer = new Timer();

    public SimpleAuto(IntakeSubsystem intakeSub, SwerveDrivetrain drivetrain){
        addRequirements(intakeSub, drivetrain);
        this.drivetrain = drivetrain;
        this.intakeSub = intakeSub;
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
    drivetrain.drive(0, 0, 0, true);
  }

  private void timedAutoSequence() {
    switch (phase){

      case 1: //drive forward to reef
       if(timer.get() < 1.175){
        //drivetrain.drive(0.5, 0, 0, false);
       } else {
        //drivetrain.drive(0, 0, 0, false);
        //phase++;
       }
       break; //end of case 1

       case 2:  //outtake coral
       if(timer.get() > 1.175 && timer.get() < 3) {
        if(timer.get() > 2.125){
          intakeSub.intakeOn();
        } 
       } else {
        intakeSub.intakeOff();
        phase++;
       }
       break; //end of case 2

      default:
        drivetrain.drive(0, 0, 0, false);
        intakeSub.intakeOff();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

*/
