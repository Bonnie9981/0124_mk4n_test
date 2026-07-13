// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ShooterSub;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.DriveSub;

// import frc.robot.Constants;

// public class ShootPoseCmd2 extends Command {
//     private final ShooterSub sh;
//     private final DriveSub drive;
    
//     private final PIDController turretPID = new PIDController(0.018, 0, 0);

//     private boolean isBlueAlliance;
//     private Pose2d targetPose;

//     public ShootPoseCmd2(ShooterSub shooter, DriveSub drive, boolean isBlueAlliance){
//         this.sh = shooter;
//         this.drive = drive;
//         this.isBlueAlliance = isBlueAlliance;
//         addRequirements(sh);

//         // turretPID.enableContinuousInput(-180, 180);
//     }

//     @Override
//     public void initialize() {
//        if (isBlueAlliance) targetPose = Constants.HUB_POSE_BLUE;
//        else targetPose = Constants.HUB_POSE_RED;
//     }

//     @Override
//     public void execute() {
//         System.out.println(targetPose);
//         var speeds = drive.getChassisSpeeds();
//         double vx = speeds.vxMetersPerSecond;
//         double vy = speeds.vyMetersPerSecond;

//         Pose2d robotPose = drive.getPose();

//         double dxNow = targetPose.getX() - robotPose.getX();
//         double dyNow = targetPose.getY() - robotPose.getY();
//         double dist = Math.hypot(dxNow, dyNow);

//         double shotTime = 1.5;

//         double futureX = robotPose.getX() + vx * shotTime;
//         double futureY = robotPose.getY() + vy * shotTime;

//         double dx = targetPose.getX() - futureX;
//         double dy = targetPose.getY() - futureY;

//         double angleToTarget = Math.atan2(dy, dx);

//         double robotHeading = robotPose.getRotation().getRadians();
//         double turretSetpoint = angleToTarget - robotHeading;

//         double turretDeg = Math.toDegrees(turretSetpoint);
//         double currentDeg = sh.getCurAngleDeg();

//         double output = turretPID.calculate(currentDeg, turretDeg);
//         output = MathUtil.clamp(output, -0.5, 0.5);

//         sh.setTurretSpeed(output);

//         double futureDist = Math.hypot(dx, dy);

//         sh.shoot(0, futureDist, vx, vy);

//         // debug
//         SmartDashboard.putNumber("turretSetpoint", turretDeg);
//         SmartDashboard.putNumber("robotX", robotPose.getX());
//         SmartDashboard.putNumber("robotY", robotPose.getY());
//         SmartDashboard.putNumber("dist", dist);
//         // debug
//         SmartDashboard.putNumber("futureX", futureX);
//         SmartDashboard.putNumber("futureY", futureY);
//         SmartDashboard.putNumber("futureDist", futureDist);
//         SmartDashboard.putNumber("turretSetpoint", turretSetpoint);
//     }

//     @Override
//     public void end(boolean interrupted){
//         sh.stopShoot();
//         sh.stopTurret();
//     }
        
//     @Override
//     public boolean isFinished(){
//         return false;
//     }
// }