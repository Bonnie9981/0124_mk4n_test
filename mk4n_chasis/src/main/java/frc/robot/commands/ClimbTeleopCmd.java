package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSub;

public class ClimbTeleopCmd extends Command {
    private final ClimberSub climber;
    private final boolean up;

    public ClimbTeleopCmd(ClimberSub climber, boolean up){
        this.climber = climber;
        this.up = up;
        addRequirements(climber);
    }

    @Override
    public void execute(){
        if (up) climber.climbUp();
        else climber.climbDown();
    }

    @Override
    public void end(boolean interrupted){
        climber.stop();
    }
        
    @Override
    public boolean isFinished(){
        return false;
    }
    
}