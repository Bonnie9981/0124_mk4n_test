package frc.robot;

import java.util.Set;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  
            COTSTalonFXSwerveConstants.SDS.MK4n.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4n.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(26.0); 
        public static final double wheelBase = Units.inchesToMeters(26.0);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics */
        public static final Translation2d flModuleOffset = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d frModuleOffset = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
        public static final Translation2d blModuleOffset = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d brModuleOffset = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            flModuleOffset, frModuleOffset, blModuleOffset, brModuleOffset
        );

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleSupplyCurrentLimit = 25;
        public static final int angleStatorCurrentLimit = 40;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveSupplyCurrentLimit = 40;
        public static final int driveStatorCurrentLimit = 80; // TODO: Drive current limit
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.1;

        /* Angle Motor PID Values */
        public static final double angleKP = 10; //10
        public static final double angleKI = 0;
        public static final double angleKD = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = SmartDashboard.getNumber("driveKp", 1); // 0.01
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0; //0.32
        public static final double driveKV = 0; //1.51
        public static final double driveKA = 0; //0.27

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = SmartDashboard.getNumber("meter_maxspeed", 3); //6.0
        /** Radians per Second */
        public static final double maxAngularVelocity = SmartDashboard.getNumber("radian_maxspeed", -3);  //6.0

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(92.119); // 187 // 97
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 23;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(17.76); //111 // 21
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 20;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(144); // -120.7 //-210.7-31.8
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {      
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 22;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(84); // 185.4 //95.4
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }



    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 5.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1.0;
        public static final double kPYController = 1.0;
        public static final double kPThetaController = 1.0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
            );
    }

    /* MotorID */
    public static final class MotorID{
        public static final int shoot1ID = 2; // neo
        public static final int shoot2ID = 1; // neo
        public static final int elevatorID = 3; // neo
        public static final int turretID = 11;
        
        public static final int armID = 4; // neo
        public static final int rollerID = 17;

        public static final int bigID = 14;
        public static final int smallID = 15; 

        public static final int climbID = 5; 
    }

    /* Speed */
    public static final class Speed {
        public static final double rollerSpeed = -0.4;
        public static final double smallSpeed = 0.4;
        public static final double bigSpeed = 0.15;   // 0.25
        public static final double shootSpeed = 0.5; // TODO: shoot speed
        public static final double climbSpeed = 0.2; // TODO: climb speed
        public static final double elevatorSpeed = 0.3; // TODO: elevator speed
        public static final double turretTeleopSpeed = 0.2;
        public static final double armTeleopSpeed = 0.05;
    }

    public static final double armTargetPos = -13; // TODO: arm pos

    
    public static final Pose2d HUB_POSE_RED = new Pose2d(
       11.9015002,//8.27, // 場地中心 X //11.2978438 12.5051566 // 5.2151534 4.007866 = 4.6115097
       4.0213534,//4.62534, // 場地中心 Y
        new Rotation2d(0)
    );

    public static final Pose2d HUB_POSE_BLUE = new Pose2d(
       4.6115097,
       4.0213534,
        new Rotation2d(0)
    );

    public static final Pose2d SHOOT_OUT_POSE_RED = new Pose2d(
       14.473365,
       0,
        new Rotation2d(0)
    );

    public static final Pose2d SHOOT_OUT_POSE_BLUE = new Pose2d(
       2.3212797,
       0,
        new Rotation2d(0)
    );

    /* DIOport */
    public static final class DIOport{
      
    }

    /* Limelight */
    public static final class Limelight{
     
    }
    
    // TODO: Gear ratio
    public static final double TURRET_GEAR_RATIO = 1.85 / 90;// 3.91 / 180; // TODO: gear ratio
    public static final double ELEVATOR_GEAR_RATIO = 10;

    public static final double Elevator_pos_limit = 10000; // TODO: elevator pos

    public static final double SHOOTER_HEIGHT = 100;

    public static final Set<Integer> red_TAGS = Set.of(2,  5, 8, 9, 10, 11); // 3, 4,
    public static final Set<Integer> blue_TAGS = Set.of(18, 21, 24, 25, 26, 27); // 19, 20

    public static final int Tag_RED = 10;
    public static final int Tag_BLUE = 26;
}