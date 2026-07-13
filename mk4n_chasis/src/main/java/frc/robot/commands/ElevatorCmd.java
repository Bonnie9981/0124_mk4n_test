package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSub;

public class ElevatorCmd extends Command {
    private final ElevatorSub el;
    private final boolean up;
    // private final PIDController elevationPID = new PIDController(0, 0, 0);

    
    public ElevatorCmd(ElevatorSub el, boolean up) {
        this.el = el;
        this.up = up;
        addRequirements(el);
    }

    // @Override
    // public void initialize() {
        
    // }

    @Override
    public void execute() {
        // if (up) el.up();
        // else el.down();
        
        // double currentAngle = el.getAngle();
        // boolean hasTarget = LimelightHelpers.getTV("limelight");

        //     if (hasTarget) {
        //         double[] poseArray = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        //         double d = poseArray[2];

        //         // double ty = LimelightHelpers.getTY("limelight");
        //         // double h = Constants.SHOOTER_HEIGHT;
        //         // double d = h / Math.tan(Math.toRadians(ty));

        //         double angleRad = el.caculateAngleRad(d);
        //         if (!Double.isNaN(angleRad)) {
        //             double targetAngleDeg = Math.toDegrees(angleRad);
        //             el.setAngle(targetAngleDeg);
        //         }
        //     }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // el.stopElevator();
    }
}