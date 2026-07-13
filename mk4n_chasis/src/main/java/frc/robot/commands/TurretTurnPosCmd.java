package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import frc.robot.Constants;

public class TurretTurnPosCmd extends Command {
    private final ShooterSub sh;
    private final double angle;
    private final PIDController initPid = new PIDController(0.4, 0, 0); // TODO: turn90 pid 0.4

    // private boolean haveToReset = false;

    public TurretTurnPosCmd(ShooterSub sh, double angle){
        this.sh = sh;
        this.angle = angle;
        addRequirements(sh);
    }

    @Override
    public void initialize(){
        // haveToReset = (angle == 90);
        // sh.resetTurret();
    }

    @Override
    public void execute() {
        double cur = sh.getPos();
        double error = angle * Constants.TURRET_GEAR_RATIO;
        sh.setTurretSpeed(initPid.calculate(cur, error)); // TODO: check +-
    }

    @Override
    public void end(boolean interrupted){
        sh.stopTurret();
        // sh.resetTurret();
    }
        
    @Override
    public boolean isFinished(){
        return initPid.atSetpoint();
    }
}