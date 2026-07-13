package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import frc.robot.Constants;

public class ZeroTurretPosCmd extends Command {
    private final ShooterSub sh;

    public ZeroTurretPosCmd(ShooterSub sh){
        this.sh = sh;
        addRequirements(sh);
    }

    @Override
    public void initialize(){
        sh.resetTurret();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted){
    }
        
    @Override
    public boolean isFinished(){
        return false;
    }
}