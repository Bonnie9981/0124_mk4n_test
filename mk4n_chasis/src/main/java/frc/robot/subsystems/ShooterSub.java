package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import static frc.robot.Constants.MotorID.*;
import static frc.robot.Constants.Speed.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;


public class ShooterSub extends SubsystemBase {
     private final SparkMax shooter1 = new SparkMax(shoot1ID, MotorType.kBrushless); // bottom grey
     private final SparkMax shooter2 = new SparkMax(shoot2ID, MotorType.kBrushless); // top green

     private final TalonFX turretMotor = new TalonFX(turretID);

     
     
     private final RelativeEncoder topEncoder;
     private final RelativeEncoder bottomEncoder;

     private final SparkClosedLoopController topPID;
     private final SparkClosedLoopController bottomPID;

     private double lastZ = 0.0;

     private double targetTopRPM = getTopSpeed(lastZ);

     private double targetBottomRPM = -2200;

     private double xxx = 0.0, zzz = 0.0;

     public ShooterSub(){

          // shooter2.setPeriodicFrameTimeout(PeriodicFrame.kStatus0, 500);
          turretMotor.setPosition(0); // 90 * Constants.TURRET_GEAR_RATIO

          // ===== Shooter 1 (Master) =====
          SparkMaxConfig shooter1Config = new SparkMaxConfig();
          FeedForwardConfig shooter1ff = new FeedForwardConfig();

          shooter1ff.kV(0.00017);

          shooter1Config.closedLoop
          .p(0.0002)
          .i(0)
          .d(0)
          .feedForward.apply(shooter1ff);

          shooter1.configure(shooter1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

          // ===== Shooter 2 (Follower) =====
          SparkMaxConfig shooter2Config = new SparkMaxConfig();

          FeedForwardConfig shooter2ff = new FeedForwardConfig();

          shooter2ff.kV(0.00017);

          shooter2Config.closedLoop
               .p(0.0002)
               .i(0)
               .d(0)
               .feedForward.apply(shooter2ff);

          shooter2.configure(shooter2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

          // Encoder
          topEncoder = shooter1.getEncoder();
          bottomEncoder = shooter2.getEncoder();
     
          // PID controller
          topPID = shooter1.getClosedLoopController();
          bottomPID = shooter2.getClosedLoopController();
     } 

     /* Shooter -----------------------------------------------------------------------*/

     public void poseShoot(double tx, double tz, double vx, double vy){
          xxx = tx;
          zzz = tz;

          tz -= 0.5969 + 0.419;

          double dist = Math.sqrt(tx*tx + tz*tz);

          double dirX = tx / dist;
          double dirZ = tz / dist;

          double robotTowardTarget =
                    vy * dirZ +
                    vx * dirX;

          double rpmComp = robotTowardTarget * 600; // 800

          targetTopRPM = getTopSpeed(dist) + rpmComp;

          // vx *= 1.2;
          // vy *= 1.2;

          // double vectorX = tx + vy; // 左右
          // double vectorY = tz + vx; // 前後

          // double new_dist = Math.sqrt(vectorX*vectorX + vectorY*vectorY);

          // double error = 0.5969 * vectorX / vectorY; // hub half
          // new_dist -= error;

          // targetTopRPM = getTopSpeed(new_dist);

          topPID.setSetpoint(targetTopRPM, SparkBase.ControlType.kVelocity);
          bottomPID.setSetpoint(targetBottomRPM, SparkBase.ControlType.kVelocity);
     }
    
          


     

     public void tagShoot(double tx, double tz, double vx, double vy){
          xxx = tx;
          zzz = tz;

          // double dist = Math.sqrt(tx*tx + tz*tz);

          // double dirX = tx / dist;
          // double dirZ = tz / dist;

          // double robotTowardTarget =
          //           vy * dirZ +
          //           vx * dirX;

          // double rpmComp = robotTowardTarget * 600; // 8000

          targetTopRPM = getTopSpeed(Math.abs(zzz-0.6)); //+ rpmComp;



          topPID.setSetpoint(targetTopRPM, SparkBase.ControlType.kVelocity);
          bottomPID.setSetpoint(targetBottomRPM, SparkBase.ControlType.kVelocity);
     }

     public void shoot_with_dist(double dist){
          targetTopRPM = getTopSpeed(Math.abs(dist));

          topPID.setSetpoint(targetTopRPM, SparkBase.ControlType.kVelocity);
          bottomPID.setSetpoint(targetBottomRPM, SparkBase.ControlType.kVelocity);
     }

     public double getTopSpeed(double dz) {
          return -480 * dz - 3120; // tag
     }

     public void stopShoot(){
         shooter1.stopMotor();
         shooter2.stopMotor();
     }

     private int stableLoops = 0;

     public boolean atSpeed() {
          double topError = Math.abs(topEncoder.getVelocity() - targetTopRPM);
          double bottomError = Math.abs(bottomEncoder.getVelocity() - targetBottomRPM);

          if (topError < 150 && bottomError < 150) {
               stableLoops++;
          } else {
               stableLoops = 0;
          }

          return stableLoops > 10;  // 20ms loop × 10 ≈ 0.2 秒
     }

     

     /* Turret --------------------------------------------------------------------------------------*/
     public void setTurretSpeed(double speed){
          turretMotor.set(speed);
     }

     public void stopTurret(){
          turretMotor.stopMotor();
     }

        public void turnRight() {
          turretMotor.set(-turretTeleopSpeed);
     }

     public void turnLeft() {
          turretMotor.set(turretTeleopSpeed);
     }

     public double getPos(){
          return turretMotor.getPosition().getValueAsDouble();
     }

     public double getCurAngleDeg() {
          return getPos() / Constants.TURRET_GEAR_RATIO; // TODO
     }

     public void resetTurret() {
          turretMotor.setPosition(0);
     }

     // public Set getHubTags() {
     //      boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
     //      if (isBlueAlliance) return Constants.blue_TAGS;
     //      else return Constants.red_TAGS;
     // }

     public Rotation2d getAngleToTarget(Pose2d robotPose,Pose2d tragetPose){
          double dx = tragetPose.getX() - robotPose.getX();
          double dy = tragetPose.getY() - robotPose.getY();

          return new Rotation2d(Math.atan2(dy,dx));
     }


     public void printLz(double z) {
          lastZ = z;
     }

     @Override
     public void periodic() {
          SmartDashboard.putNumber("Turret Pos", getPos());
          SmartDashboard.putNumber("Turret Angle", getCurAngleDeg());

          SmartDashboard.putNumber("shoot1 current", shooter1.getOutputCurrent());
          SmartDashboard.putNumber("shoot2 current", shooter2.getOutputCurrent());

          SmartDashboard.putNumber("shoot1 rpm", topEncoder.getVelocity());
          SmartDashboard.putNumber("shoot2 rpm", bottomEncoder.getVelocity());

          SmartDashboard.putNumber("shoot1 output %", shooter1.getAppliedOutput());
          SmartDashboard.putNumber("shoot2 output %", shooter2.getAppliedOutput());

          SmartDashboard.putNumber("battery v", RobotController.getBatteryVoltage());

          SmartDashboard.putNumber("Zzz", lastZ);
          SmartDashboard.putNumber("target top", targetTopRPM);

          SmartDashboard.putNumber("Z", zzz);
          SmartDashboard.putNumber("X", xxx);

          boolean turret_safe = getCurAngleDeg() >= -106 || getCurAngleDeg() < 90;
          if (!turret_safe) System.out.println("zturret up to limit.");
          SmartDashboard.putBoolean("Turret Safe", turret_safe);

          
     }
}