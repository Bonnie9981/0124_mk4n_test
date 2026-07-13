package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSub;

public class ConveyorShakeCmd extends Command {
    private final ConveyorSub cv;
    
    private final Timer timer = new Timer();

    public ConveyorShakeCmd(ConveyorSub cv){
      this.cv = cv;
      addRequirements(cv);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        double t = timer.get();

        if ((int)(t/ 0.15) % 2 == 0) {
            cv.setBigSpeed(0.3);
        } else {
            cv.setBigSpeed(-0.3);
        }
    }

    @Override
    public void end(boolean interrupted){
        cv.stopConvey();
    }
        
    @Override
    public boolean isFinished(){
        return false;
    }
}