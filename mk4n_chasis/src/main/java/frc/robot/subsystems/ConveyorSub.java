package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.MotorID.*;
import static frc.robot.Constants.Speed.*;

public class ConveyorSub extends SubsystemBase {    
     private final TalonFX  big = new TalonFX(bigID);
     private final TalonFX small = new TalonFX(smallID);
     
     public void conveySuck() {
          big.set(bigSpeed);
          small.set(smallSpeed);
     }

     public void conveySpit() {
          big.set(-bigSpeed);
     }

     public void setBigSpeed(double speed) {
          big.set(speed);
     } 

     public void setSmallSpeed(double speed) {
          small.set(speed);
     }   

     public void stopConvey() {
          big.stopMotor();
          small.stopMotor();
     }  

     public double getBigCurrent() {
        return big.getSupplyCurrent().getValueAsDouble();
     }
}