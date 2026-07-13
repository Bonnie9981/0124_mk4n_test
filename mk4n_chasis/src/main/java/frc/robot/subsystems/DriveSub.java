package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;
import java.util.Optional;

public class DriveSub extends SubsystemBase {
    // private final AHRS gyro1;
    private final AHRS gyro2;

    private final SwerveModule[] swerveModules;
    private final Field2d field = new Field2d();
    private final SwerveDrivePoseEstimator poseEstimator;

    private final PIDController headingPID = new PIDController(0.01, 0, 0.002);
    private double targetHeading = 0;

    private AprilTagFieldLayout layout;

    String[] names = new String[8];
    double[] voltages = new double[8];
    double[] supplyCurrents = new double[8];
    double[] statorCurrents = new double[8];    

    public DriveSub() {
        // gyro1 = new AHRS(NavXComType.kUSB1);
        // gyro1.reset();

        gyro2 = new AHRS(NavXComType.kUSB1); // green
        gyro2.reset();

        // targetHeading = getHeading().getDegrees();
        // headingPID.setTolerance(1.0); // ±1 deg 誤差不動
        // headingPID.setIntegratorRange(-5, 5); // 防止累積

        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // encoder trust
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5))  // vision trust
        );

        SmartDashboard.putData("Field", field);
        field.getObject("target").setPose(Constants.HUB_POSE_BLUE); // TODO
        

        headingPID.enableContinuousInput(-180, 180);

        configureAutoBuilder();
        resetModulesToAbsolute();

        // layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);

        // var tagPose = layout.getTagPose(3);

        // SmartDashboard.putNumber("3", tagPose.get().getX());
        // SmartDashboard.putNumber("3", tagPose.get().getY());


        try {
            // 載入場地配置
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
            
            // 嘗試取得 3 號 Tag (請確認 ID 是否正確)
            var tagPose20 = layout.getTagPose(20);
            
            if (tagPose20.isPresent()) {
                SmartDashboard.putNumber("Tag 20 X", tagPose20.get().getX());
                SmartDashboard.putNumber("Tag 20 Y", tagPose20.get().getY());
            } else {
                DriverStation.reportError("Layout loaded but Tag 20 not found!", false);
            }

            // 嘗試取得 3 號 Tag (請確認 ID 是否正確)
            var tagPose26 = layout.getTagPose(26);
            
            if (tagPose26.isPresent()) {
                SmartDashboard.putNumber("Tag 26 X", tagPose26.get().getX());
                SmartDashboard.putNumber("Tag 26 Y", tagPose26.get().getY());
            } else {
                DriverStation.reportError("Layout loaded but Tag 26 not found!", false);
            }

        } catch (java.io.IOException e) {
            // 如果讀取失敗，印出錯誤並給一個警告
            DriverStation.reportError("Failed to load AprilTagFieldLayout: " + e.getMessage(), e.getStackTrace());
            layout = null; 
        }

    }

    private void configureAutoBuilder() {
        RobotConfig config = null;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("⚠️ PathPlanner config load failed");
        }
        

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::driveRobotRelative,
            new PPHolonomicDriveController(
                new PIDConstants(5, 0, 0),
                new PIDConstants(0, 0, 0)
            ),
            config,
            () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                    == DriverStation.Alliance.Red,
            this
        );
    }

    @Override
    public void periodic() {
        updateOdometry();

        // SmartDashboard.putNumber("Heading", getHeading().getDegrees());
        // SmartDashboard.putNumber("Heading 2", -gyro1.getYaw());
        SmartDashboard.putNumber("Heading 2", -gyro2.getYaw());
        // SmartDashboard.putBoolean("gyro 1 Connected", gyro1.isConnected());
        SmartDashboard.putBoolean("gyro 2 Connected", gyro2.isConnected());

        field.setRobotPose(getPose()); 
        SmartDashboard.putNumber("poseX", getPose().getX());
        SmartDashboard.putNumber("poseY", getPose().getY());

        // for (SwerveModule mod : swerveModules) {
        //     Rotation2d angle = mod.getCANcoder();
        //     System.out.println("Module " + mod.moduleNumber + " CANcoder: " + angle.getDegrees());
        // }

        for (SwerveModule mod : swerveModules) {
            int i = mod.moduleNumber;
            names[i] = "Mod" + i + " Drive";
            voltages[i] = mod.getDriveVoltage();
            supplyCurrents[i] = mod.getDriveSupplyCurrent();
            statorCurrents[i] = mod.getDriveStatorCurrent();
        }

        for (SwerveModule mod : swerveModules) {
            int i = mod.moduleNumber + 4;
            names[i] = "Mod" + i + " Angle";
            voltages[i] = mod.getAngleVoltage();
            supplyCurrents[i] = mod.getAngleSupplyCurrent();
            statorCurrents[i] = mod.getAngleStatorCurrent();
        }

        SmartDashboard.putStringArray("Motor Names", names);
        SmartDashboard.putNumberArray("Motor Voltages", voltages);
        SmartDashboard.putNumberArray("Motor Supply Currents", supplyCurrents);
        SmartDashboard.putNumberArray("Motor Stator Currents", statorCurrents);

        SmartDashboard.putNumber("X speed", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Y speed", getChassisSpeeds().vyMetersPerSecond);
    }

    public void updateOdometry() {

        
        poseEstimator.update(getHeading(), getModulePositions());

        

        LimelightHelpers.SetRobotOrientation(
            "limelight",
            getHeading().getDegrees(),
            0, 0, 0, 0, 0
        );

        var result = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");


        // filtering
        if (result != null && result.tagCount >= 2) {// && result.tagCount >= 2 && result.avgTagDist < 4.0

            poseEstimator.addVisionMeasurement(
                result.pose,
                result.timestampSeconds
            );
        }

       

        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        // if ((Math.abs(translation.getX()) > 0.1 || Math.abs(translation.getY()) > 0.1) && Math.abs(rotation) < 0.1) {
        //     // 沒在轉 → 鎖角度
        //     rotation = headingPID.calculate(getHeading().getDegrees(),targetHeading);
        //     rotation = MathUtil.clamp(rotation, -Units.degreesToRadians(90), Units.degreesToRadians(90));
        // } else if (Math.abs(rotation) > 0.2) {
        //     // 有轉 → 更新目標角度
        //     targetHeading = getHeading().getDegrees();
        // }

        ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation);

        SwerveModuleState[] states =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                states,
                Constants.Swerve.maxSpeed
        );

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(states[mod.moduleNumber], false);
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive(
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
            speeds.omegaRadiansPerSecond,
            false
        );
    }

    // --- Helper Methods ---

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Rotation2d getHeading() {
        // Note: Check if your gyro1 needs to be negated based on your drive orientation
        if (gyro2.isConnected()) 
            return Rotation2d.fromDegrees(-gyro2.getYaw());
        // else if (gyro1.isConnected())
        //     return Rotation2d.fromDegrees(-gyro1.getYaw());
            
        return Rotation2d.fromDegrees(0.0);
        
        // return Rotation2d.fromDegrees(-gyro1.getYaw()); // + 180 // TODO: gyro1 yaw
    }

    public void zeroHeading() {
        // gyro1.reset();
        gyro2.reset();
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public boolean isBlueAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        }

        return true; // fallback
    }
}