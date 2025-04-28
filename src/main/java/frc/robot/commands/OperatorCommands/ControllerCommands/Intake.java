package frc.robot.commands.OperatorCommands.ControllerCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
    private IntakeSubsystem subsystem;
    private Supplier<Boolean> intake;
    //boolean intakeOn = false;

    public Intake(IntakeSubsystem subsystem, Supplier<Boolean> intake){
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.intake = intake;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if(intake.get()){
            subsystem.intakeOn();
        } else{
            subsystem.intakeOff();
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
