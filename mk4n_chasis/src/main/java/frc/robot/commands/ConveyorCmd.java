package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.ShooterSub;

public class ConveyorCmd extends Command {
    private final ConveyorSub cv;
    private final ShooterSub sh;
    private final boolean suck;

    private final Timer timer = new Timer();

    public ConveyorCmd(ConveyorSub cv, ShooterSub sh, boolean suck){
      this.cv = cv;
      this.sh = sh;
      this.suck = suck;
      addRequirements(cv);
    }

  @Override
    public void initialize(){
        timer.reset();
        timer.start();
  }

  @Override
  public void execute(){
    // if (!sh.atSpeed()) {
    //     cv.stopConvey();
    //     return;
    // }
  double t = timer.get();

    if (suck) {
      double cycleTime = t % 5.5;

      if (cycleTime < 5.0) {
          // 正常 convey
          cv.conveySuck();
      } 
      else {
          // shake
          cv.setSmallSpeed(0.4);

          if ((int)(t / 0.15) % 2 == 0) {
            cv.setBigSpeed(0.3);
          } else {
            cv.setBigSpeed(-0.25);
          }
        }
    }

    else 
      cv.conveySpit();
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