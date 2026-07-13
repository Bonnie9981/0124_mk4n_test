package frc.robot.commands;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.ShooterSub;

public class ShootDistCmd extends Command {
    private final ShooterSub sh;
    private final double dist;

    public ShootDistCmd(ShooterSub sh, double dist){
        this.sh = sh;
        this.dist = dist;
        addRequirements(sh);
    }

    @Override
    public void execute(){
        sh.shoot_with_dist(dist);
    }

    @Override
    public void end(boolean interrupted){
        sh.stopShoot();
    }
        
    @Override
    public boolean isFinished(){
        return false;
    }
}