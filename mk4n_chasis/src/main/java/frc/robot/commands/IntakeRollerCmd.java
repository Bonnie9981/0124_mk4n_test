package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

public class IntakeRollerCmd extends Command {
    private final IntakeSub in;
    private final boolean suck;

    public IntakeRollerCmd(IntakeSub in, boolean suck){
        this.in = in;
        this.suck = suck;
        addRequirements(in);
    }

    @Override
    public void execute(){
        if (suck) in.rollSuck();
        else in.rollSpit();
    }

    @Override
    public void end(boolean interrupted){
        in.stopRoller();
    }
        
    @Override
    public boolean isFinished(){
        return false;
    }
}