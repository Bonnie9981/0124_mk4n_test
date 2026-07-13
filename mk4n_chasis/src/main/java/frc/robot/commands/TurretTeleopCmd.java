package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class TurretTeleopCmd extends Command {
    private final ShooterSub sh;
    private final boolean right;

    public TurretTeleopCmd(ShooterSub sh, boolean right){
        this.sh = sh;
        this.right = right;
        addRequirements(sh);
    }

    @Override
    public void execute(){
        if (right) sh.turnRight();
        else sh.turnLeft();
    }

    @Override
    public void end(boolean interrupted){
        sh.stopTurret();
    }
        
    @Override
    public boolean isFinished(){
        return false;
    }
}