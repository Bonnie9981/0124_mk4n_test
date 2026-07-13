package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSub;

public class DriveTeleopCmd extends Command {
    private final DriveSub driveSubsystem;
    private final DoubleSupplier xSpeed, ySpeed, rot;
    private BooleanSupplier fieldRelative;

    public DriveTeleopCmd(DriveSub subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, BooleanSupplier fieldRelative) {
        this.driveSubsystem = subsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.fieldRelative = fieldRelative;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (fieldRelative.getAsBoolean())
            driveSubsystem.drive(new Translation2d(-xSpeed.getAsDouble(), ySpeed.getAsDouble()), rot.getAsDouble(), true);
        else 
            driveSubsystem.drive(new Translation2d(-xSpeed.getAsDouble(), ySpeed.getAsDouble()), rot.getAsDouble(), false);

        SmartDashboard.putNumber("xspeed", xSpeed.getAsDouble());
        SmartDashboard.putNumber("yspeed", ySpeed.getAsDouble());
        SmartDashboard.putNumber("rotspeed", rot.getAsDouble());
    }
}