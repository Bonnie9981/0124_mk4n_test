package frc.robot.subsystems;

import static frc.robot.Constants.MotorID.*;
import static frc.robot.Constants.Speed.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSub extends SubsystemBase {
     private final SparkMax climb = new SparkMax(climbID, MotorType.kBrushless);
     private final double speed = climbSpeed;
 
     private final PIDController pidController;
     private final RelativeEncoder encoder;

    public ClimberSub() {
        pidController = new PIDController(0.01, 0, 0); // TODO
        encoder = climb.getEncoder();
        resetEncoder();
    }

    public void upPos(){
        climb.set(pidController.calculate(getPos(), 70)); // TODO
    }

    public void downPos(){
        climb.set(-0.5);
        // climb.set(pidController.calculate(getPos(), -70));
    }

    public void climbUp(){
        climb.set(speed);
    }

    public void climbDown() {
        climb.set(-speed);
    }

    public void stop() {
        climb.stopMotor();
    }

    public double getPos() {
        return encoder.getPosition();
    }
    
    public void resetEncoder() {
        encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climb pos", encoder.getPosition());
    }
}