package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

public class ArmTeloepCmd extends Command {
    private final IntakeSub in;
    private final boolean up;

    public ArmTeloepCmd(IntakeSub in, boolean up){
        this.in = in;
        this.up = up;
        addRequirements(in);
    }

    @Override
    public void execute(){
        if (up) in.up();
        else in.down();
    }

    @Override
    public void end(boolean interrupted){
        in.stopArm();
    }
        
    @Override
    public boolean isFinished(){
        return false;
    }
}