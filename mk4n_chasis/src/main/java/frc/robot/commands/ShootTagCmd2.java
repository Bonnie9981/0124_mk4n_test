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

public class ShootTagCmd2 extends Command {
    private final ShooterSub sh;
    private final DriveSub drive;
    
    private final PIDController turretPID = new PIDController(0.018, 0, 0);

    private boolean isBlueAlliance;
    private int targetTagID;
    private RawFiducial lockedTarget;

    public ShootTagCmd2(ShooterSub shooter, DriveSub drive, boolean isBlueAlliance){
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
        RawFiducial[] tags = LimelightHelpers.getRawFiducials("limelight");

        if (tags == null || tags.length == 0) {
            sh.stopTurret();
            return;
        }

        for (var tag : tags) {
            if (tag.id == 10 || tag.id == 26) { // targetTagID
                lockedTarget = tag;
                break;
            }
        }

        double tx = lockedTarget.txnc;
        double z = lockedTarget.distToCamera;
        
        // || sh.getCurAngleDeg() + tx >= 90 || sh.getCurAngleDeg() + tx <= -90
        if (Math.abs(tx) < 1) {
            sh.stopTurret();
        } else {
            double output = turretPID.calculate(tx, 0);
            output = MathUtil.clamp(output, -0.4, 0.4);
            sh.setTurretSpeed(output);
        }

        sh.shoot_with_dist(z);
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