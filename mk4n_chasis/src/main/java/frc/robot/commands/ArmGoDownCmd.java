package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

public class ArmGoDownCmd extends Command {
    private final IntakeSub in;
    private final double time;

    private final Timer timer = new Timer();

    public ArmGoDownCmd(IntakeSub in, double time){
        this.in = in;
        this.time = time;
        addRequirements(in);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        double t = timer.get();
        if (t < time) in.setArmSpeed(-0.2);
    }

    @Override
    public void end(boolean interrupted){
        in.setArmSpeed(0);
    }
        
    @Override
    public boolean isFinished(){
        return timer.get() >= time;
    }
}