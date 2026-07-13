package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

//import com.pathplanner.lib.events.EventTrigger;

public class RobotContainer {
   private SendableChooser<Command> autoChooser = new SendableChooser<>();
   private final SendableChooser<Boolean> AllianceChooser = new SendableChooser<>();
    //joystick
    private final Joystick joy1 = new Joystick(0);
    private final Joystick joy2 = new Joystick(1);

    /* Sub */
    private final DriveSub drive;
    private final ElevatorSub elevator;
    private final ShooterSub shooter;
    private final IntakeSub intake;
    private final ConveyorSub conveyor;
    private final ClimberSub climber;

    private double shoot_dist = 0.0;

    private double applyDeadband(double speed) {
        return Math.abs(speed) > 0.1 ? speed : 0.0;
    }

    /* Chassis */
    private double getxyAdjustSpeed() {
        return 5 - 2 * applyDeadband(joy1.getRawAxis(2)); // trigger // 4.5 2.5
    }

    private double getrotAdjustSpeed() {
        return 4.5 - 1.5 * applyDeadband(joy1.getRawAxis(2));
    }

    private double getXSpeed() {
        double speed = getxyAdjustSpeed();
        int pov = joy1.getPOV();

        if (pov != -1) {
            double rad = Math.toRadians(pov);
            return Math.sin(rad) * speed;
        }

        return applyDeadband(joy1.getRawAxis(0)) * speed;
    } 

    private double getYSpeed() {
        double speed = getxyAdjustSpeed();
        int pov = joy1.getPOV();

        if (pov != -1) {
            double rad = Math.toRadians(pov);
            return -Math.cos(rad) * speed;
        }

        return applyDeadband(joy1.getRawAxis(1)) * speed;
    }

    private double getRotSpeed() {
        return applyDeadband(joy1.getRawAxis(4)) * getrotAdjustSpeed(); // 3
    }

    private boolean isFieldMode() {
        return joy1.getPOV() == -1;
    }

    public RobotContainer(DriveSub drive, ElevatorSub elevator, ShooterSub shooter, ConveyorSub conveyor, IntakeSub intake, ClimberSub climber) {
        this.drive = drive;
        this.elevator = elevator;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.intake = intake;
        this.climber = climber;

        /*alliance chooser */
        AllianceChooser.setDefaultOption("Blue Alliance", true);
        AllianceChooser.addOption("Red Alliance", false);
        SmartDashboard.putData("Alliance Color", AllianceChooser);

        /* auto */
        NamedCommands.registerCommand("intake_out", new ArmTeloepCmd(intake, false));
        NamedCommands.registerCommand("intake_down", new ArmGoDownCmd(intake, 1.5));
        NamedCommands.registerCommand("climb", new ClimbTeleopCmd(climber, true));

        NamedCommands.registerCommand("turret_turn45", new TurretTurnPosCmd(shooter, -45));
        NamedCommands.registerCommand("shoot_1.92", new ShootDistCmd(shooter, 2.73) //2.73 - 0.844 -0.5986 // 2.499 // 
                 .alongWith( new WaitCommand(1)
                 .andThen(new ConveyorCmd(conveyor, shooter, true)))); // hub center //1.92
    
        NamedCommands.registerCommand("shoot_1.5", new ShootDistCmd(shooter, 1.5)
                 .alongWith( new WaitCommand(1)
                 .andThen(new ConveyorCmd(conveyor, shooter, true)))); // hub center //1.92

        NamedCommands.registerCommand("turret_turn40.9", new TurretTurnPosCmd(shooter, 27));
        NamedCommands.registerCommand("shoot_1.98", new ShootDistCmd(shooter, 1.98)//2.73 - 0.8yt44
                 .alongWith( new WaitCommand(1)
                 .andThen(new ConveyorCmd(conveyor, shooter, true)))); // hub center //1.92

        /* drive */
        drive.setDefaultCommand(new DriveTeleopCmd(drive, this::getXSpeed, this::getYSpeed, this::getRotSpeed, this::isFieldMode));
    
        configureButtonBindings();  
        
        /*auto chooser */
        // autoChooser = AutoBuilder.buildAutoChooser("blue_Tests");
        


        
       



        

        SmartDashboard.putData("Auto mode", autoChooser);
        autoChooser.addOption("blue_climb", AutoBuilder.buildAuto("blue_climb"));
        autoChooser.addOption("preload_l", AutoBuilder.buildAuto("preload_l"));
        autoChooser.addOption("preload_m", AutoBuilder.buildAuto("preload_m"));
        autoChooser.addOption("preload_r", AutoBuilder.buildAuto("preload_r"));
        autoChooser.addOption("bump_preload_l", AutoBuilder.buildAuto("bump_preload_l"));
        autoChooser.addOption("bump_preload_m", AutoBuilder.buildAuto("bump_preload_m"));
        autoChooser.addOption("bump_preload_r", AutoBuilder.buildAuto("bump_preload_r"));
        autoChooser.addOption("test", AutoBuilder.buildAuto("bump_test"));




        getAutonomousCommand();
    }
    

    private void configureButtonBindings() {
        // Driver 1 ----------------------------------------------------------------------

        /* Roller in */
        new JoystickButton(joy1, 2).toggleOnTrue(new IntakeRollerCmd(intake, true)
            .beforeStarting(() -> {SmartDashboard.putBoolean("Intake", true);})
            .finallyDo(() -> {SmartDashboard.putBoolean("Intake", false);})
        );

        /* Roller out */
        new JoystickButton(joy1, 3).toggleOnTrue(
            new IntakeRollerCmd(intake, false).alongWith(new ConveyorCmd(conveyor, shooter, false))
            .beforeStarting(() -> {SmartDashboard.putBoolean("Roll Out", true);})
            .finallyDo(() -> {SmartDashboard.putBoolean("Roll Out", false);})
        );        

        /* Climb Pos */
        new JoystickButton(joy1, 6).onTrue(new ClimbPosCmd(climber, true));
        new JoystickButton(joy1, 5).whileTrue(new ClimbPosCmd(climber, false)); // fast down

        // Climb Teleop
        new JoystickButton(joy1, 4).whileTrue(new ClimbTeleopCmd(climber, true));
        new JoystickButton(joy1, 1).whileTrue(new ClimbTeleopCmd(climber, false));

        /* Gyro reset */ 
        new JoystickButton(joy1, 7).onTrue(new ZeroHeadingCmd(drive));


        // Driver 2 ---------------------------------------------------------------------------------

        /* shoot & convey */
        // S1
        // new JoystickButton(joy2, 4).toggleOnTrue(
        //     new ShootAimCmd(shooter, drive, getShooterRatio())
        //         .alongWith(
        //             new WaitUntilCommand(() -> shooter.atSpeed())
        //                 .andThen(new ConveyorCmd(conveyor, shooter, true))
        //         )
        // );

        // S2
        new JoystickButton(joy2, 4).toggleOnTrue(
           
                new ShootPoseCmd(shooter, drive, isBlueAlliance())
                .alongWith( new WaitCommand(1)
                .andThen(new ConveyorCmd(conveyor, shooter, true)))

            
            .beforeStarting(() -> SmartDashboard.putBoolean("Shoot", true))
            .finallyDo(() -> SmartDashboard.putBoolean("Shoot", false))
        );

        new JoystickButton(joy2, 3).toggleOnTrue(
            
                new ShootTagCmd(shooter, drive, isBlueAlliance())
                .alongWith( new WaitCommand(1)
                .andThen(new ConveyorCmd(conveyor, shooter, true)))

            .beforeStarting(() -> SmartDashboard.putBoolean("Shoot", true))
            .finallyDo(() -> SmartDashboard.putBoolean("Shoot", false))
        );

         



        new JoystickButton(joy2, 1).toggleOnTrue(
           
                new ShootPose_outCmd(shooter, drive, isBlueAlliance())
                .alongWith( new WaitCommand(1)
                .andThen(new ConveyorCmd(conveyor, shooter, true)))

            .beforeStarting(() -> SmartDashboard.putBoolean("Shoot", true))
            .finallyDo(() -> SmartDashboard.putBoolean("Shoot", false))
        );

        new JoystickButton(joy2, 2).toggleOnTrue(
           
               new ShootDistCmd(shooter, 2.5)
                .alongWith( new WaitCommand(1)
                .andThen(new ConveyorCmd(conveyor, shooter, true)))

            .beforeStarting(() -> SmartDashboard.putNumber("Shoot", 2.5))
            .finallyDo(() -> SmartDashboard.putNumber("Shoot", 0))
        );

        new JoystickButton(joy2, 5).toggleOnTrue(
           
               new ShootDistCmd(shooter, 3)
                .alongWith( new WaitCommand(1)
                .andThen(new ConveyorCmd(conveyor, shooter, true)))

            .beforeStarting(() -> SmartDashboard.putNumber("Shoot Dist", 2))
            .finallyDo(() -> SmartDashboard.putNumber("Shoot Dist", 0))
        );

        new JoystickButton(joy2, 6).onTrue(new TurretTurnPosCmd(shooter, 0));


        // new JoystickButton(joy2, 6).toggleOnTrue(
           
        //        new ShootDistCmd(shooter, 3)
        //         .alongWith( new WaitCommand(1)
        //         .andThen(new ConveyorCmd(conveyor, shooter, true)))

        //     .beforeStarting(() -> SmartDashboard.putNumber("Shoot Dist", 3))
        //     .finallyDo(() -> SmartDashboard.putNumber("Shoot Dist", 0))
        // );
        

        // new JoystickButton(joy2, 2).toggleOnTrue(
        //    new ShootDistCmd(shooter, 1.5)
        //         .alongWith(new ConveyorCmd(conveyor, shooter, true))

            
        //     .beforeStarting(() -> SmartDashboard.putNumber("Shoot Dist", 1.5))
        // );

        // new JoystickButton(joy2, 2).toggleOnTrue(
        //    new ShootDistCmd(shooter, 5)
        //         .alongWith(new ConveyorCmd(conveyor, shooter, true))

        //     .beforeStarting(() -> SmartDashboard.putNumber("Shoot Dist", 5))
        // );

        /* Conveyor shake */
        new JoystickButton(joy2, 10).whileTrue(new ConveyorShakeCmd(conveyor));

        
        /* Shooter Teleop */
        // Elevator
        // new POVButton(joy2, 0).whileTrue(new ElevatorCmd(elevator, true));
        // new POVButton(joy2, 180).whileTrue(new ElevatorCmd(elevator, false));
        // Turret
        new POVButton(joy2, 90).whileTrue(new TurretTeleopCmd(shooter, true));
        new POVButton(joy2, 270).whileTrue(new TurretTeleopCmd(shooter, false));

        // /* Arm Pos */
        // new JoystickButton(joy2, 5).onTrue(new ArmPosCmd(intake, true));
        // // new JoystickButton(joy2, 5).onTrue(new ArmGoDownCmd(intake, 1.5));
        // new JoystickButton(joy2, 6).onTrue(new ArmPosCmd(intake, false));

        // Arm Teleop
        new JoystickButton(joy2, 8).whileTrue(new ArmTeloepCmd(intake, true));
        new JoystickButton(joy2, 7).whileTrue(new ArmTeloepCmd(intake, false));

        // // Turret Pos
        // new JoystickButton(joy2, 10).onTrue(new TurretTurnPosCmd(shooter, -10));

        /* Turret reset */ 
        new JoystickButton(joy2, 9).onTrue(new ZeroTurretPosCmd(shooter));

    }
    

    public Command getAutonomousCommand(){
        return autoChooser.getSelected();
    }

    public boolean isBlueAlliance() {
        Boolean selected = AllianceChooser.getSelected();
        return selected != null ? selected : true; // 預設 Blue
    }
}


// alongWith(): stop when all the commands in the group stop
// raceWith(): stop when one command in the group stops