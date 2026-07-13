package frc.robot;

import java.io.IOException;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class Robot extends TimedRobot {
    private Command m_autonomousCmd; 

    public static CTREConfigs ctreConfigs = new CTREConfigs();

    public static DriveSub m_drive = new DriveSub();
    public static ElevatorSub m_elevator = new ElevatorSub();
    public static ShooterSub m_shooter = new ShooterSub();
    public static ConveyorSub m_conveyor = new ConveyorSub();
    public static IntakeSub m_intake = new IntakeSub();
    public static ClimberSub m_climber = new ClimberSub();

    public static TurretTurnPosCmd turret90 = new TurretTurnPosCmd(m_shooter, -90); 
    
    public static RobotContainer m_robotContainer;
  
    Thread m_visionThread;

    @Override
    public void robotInit() {
        m_robotContainer  = new RobotContainer(m_drive, m_elevator, m_shooter, m_conveyor, m_intake, m_climber);
        m_drive.zeroHeading(); // reset gyro heading
        // m_turret.setDefaultCommand(turret90);
     
        // // get usbcamera from CameraServer
        // UsbCamera camera = CameraServer.startAutomaticCapture(0);
        // //set the resolution
        // camera.setResolution(640, 480);
        // camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        // //get a CVSink to capture mats from the camera
        // CvSink cvSink = CameraServer.getVideo();
        // //setup a CVSource to send images to Dashboard
        // CvSource outputStream = CameraServer.putVideo("rectangle", 640, 480);
    }
    @Override
    public void autonomousInit() {
        m_autonomousCmd = m_robotContainer.getAutonomousCommand();
        if(m_autonomousCmd != null) CommandScheduler.getInstance().schedule(m_autonomousCmd);

    //    String pathName = "blue_test_shoot";
   

    // PathPlannerPath path = null;
    // try {
    //     path = PathPlannerPath.fromPathFile("blue_test_shoot"); // 從檔案讀路徑

    // } catch (FileVersionException | IOException | ParseException e) {
    //     e.printStackTrace();
    //     DriverStation.reportError("Failed to load PathPlanner path: " + "blue_test_shoot", false);
    // }

    // if (path != null) {
    //     // PathPlanner 2024+ 版本：
    //     Optional<Pose2d> startingPose;
    //     try {
    //         startingPose = path.getStartingHolonomicPose(); // 舊版或新版都可
    //     } catch (NoSuchMethodError ex) {
    //         // fallback，舊版本可能需要 path.getStartPose() 或 path.getInitialPose()
    //         startingPose = path.getStartPose(); 
    //     }

    //     if (startingPose != null) {
    //         m_drive.setPose(startingPose); // 設定 odometry
    //     } else {
    //         DriverStation.reportError("Starting Pose is null for path: " + pathName, false);
    //     }
    // }

    // // 選擇 auto command 並啟動
    // Command autoCmd = AutoBuilder.buildAuto(pathName);
    // if (autoCmd != null) {
    //     autoCmd.schedule();
    // }
}
    

    @Override
    public void autonomousPeriodic() {
        m_drive.updateOdometry();
    }

    @Override
    public void teleopInit() {
        if(m_autonomousCmd != null) m_autonomousCmd.cancel();
    }

    @Override
    public void teleopPeriodic() {
        m_drive.updateOdometry();
    }

    @Override
    public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    }
}