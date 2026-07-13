package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSub;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class ShootTagCmd extends Command {
    private final ShooterSub sh;
    private final DriveSub drive;
    
    private final PIDController turretPID = new PIDController(0.01, 0, 0);

    private boolean isBlueAlliance;
    private int targetTagID;
    private RawFiducial lockedTarget;

    public ShootTagCmd(ShooterSub shooter, DriveSub drive, boolean isBlueAlliance){
        this.sh = shooter;
        this.drive = drive;
        this.isBlueAlliance = isBlueAlliance;
        addRequirements(sh);
    }

    @Override
    public void initialize() {
       if (isBlueAlliance) targetTagID = Constants.Tag_BLUE;
       else targetTagID = Constants.Tag_RED;
    }

    @Override
    public void execute() {
        double vx = drive.getChassisSpeeds().vxMetersPerSecond;
        double vy = drive.getChassisSpeeds().vyMetersPerSecond;

        // double tx = LimelightHelpers.getTX("limelight");
        // turretPID.setSetpoint(0);
        // double output = turretPID.calculate(tx);
        // output = MathUtil.clamp(output, -0.5, 0.5);
        // sh.setTurretSpeed(-output);

        double[] poseArray = LimelightHelpers.getTargetPose_RobotSpace("limelight");

        if (poseArray != null && poseArray.length >= 3) {
            double lastX = poseArray[0];
            double lastZ = poseArray[2];

            sh.tagShoot(lastX, lastZ, vx, vy);
        }
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