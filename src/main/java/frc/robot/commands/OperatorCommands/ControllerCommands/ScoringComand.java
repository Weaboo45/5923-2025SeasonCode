package frc.robot.commands.OperatorCommands.ControllerCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class ScoringComand extends Command {
    private ScoringSubsystem subsystem;
    private Supplier<Boolean> intake, outtake, rightBumper, leftBumper;
    private Supplier<Double> leftTrigger, rightTrigger;

    public ScoringComand(ScoringSubsystem subsystem, Supplier<Boolean> outtake, Supplier<Boolean> intake,
     Supplier<Double> leftTrigger, Supplier<Double> rightTrigger,
     Supplier<Boolean> rightBumper, Supplier<Boolean> leftBumper){
        addRequirements(subsystem);
        this.subsystem = subsystem;

        this.intake = intake;
        this.outtake = outtake;

        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;

        this.rightBumper = rightBumper;
        this.leftBumper = leftBumper;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        /* 
        if(intake.get()){
            subsystem.coralOut();
        } else{
            if(outtake.get()){
                subsystem.algaeOut();
            } else{
                subsystem.intakeOff();
            }
        }

        if(rightBumper.get()){
            subsystem.armUp();
        } else {
            if(leftBumper.get()){
                subsystem.armDown();
            } else {
                subsystem.armStop();
            }
        }
            */
        

        double climbSpeed = rightTrigger.get() - leftTrigger.get();
        if(climbSpeed < 0){
            subsystem.climbDown();
        } else {
            if(climbSpeed > 0){
                subsystem.climbUp();
            } else {
                subsystem.climbStop();
            }
            
        }
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
