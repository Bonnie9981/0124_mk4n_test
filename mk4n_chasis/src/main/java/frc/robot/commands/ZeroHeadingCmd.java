package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSub;

public class ZeroHeadingCmd extends Command {
    private final DriveSub driveSubsystem;

    public ZeroHeadingCmd(DriveSub subsystem) {
        this.driveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.zeroHeading();
    }

    @Override
    public void end(boolean interrupted){
        return;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driveSubsystem.getHeading().getDegrees()) < 1.0;
    }
}