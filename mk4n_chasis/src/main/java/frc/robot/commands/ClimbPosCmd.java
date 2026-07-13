package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSub;

public class ClimbPosCmd extends Command {
    private final ClimberSub climber;
    private final boolean up;

    public ClimbPosCmd(ClimberSub climber, boolean up){
        this.climber = climber;
        this.up = up;
        addRequirements(climber);
    }

    @Override
    public void execute(){
        if (up) climber.upPos();
        else climber.downPos();
    }

    @Override
    public void end(boolean interrupted){
        climber.stop();
        climber.resetEncoder();
    }
        
    @Override
    public boolean isFinished(){
        return false;
    }
    
}