package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DriveSub;

import frc.robot.Constants;

public class ShootPoseCmd extends Command {
    private final ShooterSub sh;
    private final DriveSub drive;
    
    private final PIDController turretPID = new PIDController(0.023, 0, 0); // 0.018

    private boolean isBlueAlliance;
    private Pose2d targetPose;

    public ShootPoseCmd(ShooterSub shooter, DriveSub drive, boolean isBlueAlliance){
        this.sh = shooter;
        this.drive = drive;
        this.isBlueAlliance = isBlueAlliance;
        addRequirements(sh);

        // turretPID.enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize() {
       if (isBlueAlliance) targetPose = Constants.HUB_POSE_BLUE;
       else targetPose = Constants.HUB_POSE_RED;
    }

    @Override
    public void execute() {
        var speeds = drive.getChassisSpeeds();
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;

        /* Pose -------------------------------------------------------------------------------------------------------- */

        Pose2d robotPose = drive.getPose();

        double dx = targetPose.getX() - robotPose.getX(); // field 往紅 X, 往左 Y
        double dy = targetPose.getY() - robotPose.getY();
        
        Rotation2d targetFieldAngle = sh.getAngleToTarget(robotPose, targetPose);
        Rotation2d robotHeading = robotPose.getRotation();
        Rotation2d turretSetpoint = targetFieldAngle.minus(robotHeading);

        // if (turretSetpoint.getDegrees() >= -106 || turretSetpoint.getDegrees() < 90) {
            double currentTurretAngle = sh.getCurAngleDeg();
            double output = turretPID.calculate(currentTurretAngle, turretSetpoint.getDegrees());
            output = MathUtil.clamp(output, -0.5, 0.5);
            sh.setTurretSpeed(output);
        // }

        sh.poseShoot(Math.abs(dx), Math.abs(dy), vx, vy); // dx = 前後差, dy = 左右差
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