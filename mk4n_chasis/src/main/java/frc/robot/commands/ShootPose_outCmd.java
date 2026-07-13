package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DriveSub;

import frc.robot.Constants;

public class ShootPose_outCmd extends Command {
    private final ShooterSub sh;
    private final DriveSub drive;
    
    private final PIDController turretPID = new PIDController(0.018, 0, 0); // 0.018

    private boolean isBlueAlliance;
    private Pose2d targetPose;

    public ShootPose_outCmd(ShooterSub shooter, DriveSub drive, boolean isBlueAlliance){
        this.sh = shooter;
        this.drive = drive;
        this.isBlueAlliance = isBlueAlliance;
        addRequirements(sh);

        // turretPID.enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize() {
       if (isBlueAlliance) targetPose = Constants.SHOOT_OUT_POSE_BLUE;
       else targetPose = Constants.SHOOT_OUT_POSE_RED;
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();

        double dx = targetPose.getX() - robotPose.getX();

        sh.shoot_with_dist(Math.abs(dx) / 3);
    }

    @Override
    public void end(boolean interrupted){
        sh.stopShoot();
        sh.stopTurret();
    }
        
    @Override
    public boolean isFinished(){
        return false;
    }
}