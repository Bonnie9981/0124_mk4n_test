package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.MotorID.*;
import static frc.robot.Constants.Speed.*;

import frc.robot.Constants;
import frc.robot.Constants.*;

public class ElevatorSub extends SubsystemBase {
    // private final SparkMax elevatorMotor = new SparkMax(elevatorID, MotorType.kBrushless);
    // // private final PIDController pidController;
    // private final RelativeEncoder encoder;

    // public ElevatorSub() {
    //     // pidController = new PIDController(0, 0, 0);
    //     // encoder = elevatorMotor.getEncoder();
    //     // resetEncoder();
    // }

    // public void up() {
    //     elevatorMotor.set(elevatorSpeed);
    // }

    // public void down() {
    //     elevatorMotor.set(-elevatorSpeed);
    // }

    // public double getPos() {
    //     return encoder.getPosition();
    // }

    // // public double getAngle() {
    // //     return (encoder.getPosition() / Constants.ELEVATOR_GEAR_RATIO) * 360;
    // // }

    // // public void setAngle(double degrees) {
    // //     pidController.setSetpoint(degrees);
    // // }

    // // public double caculateAngleRad(double d) { // velocity, distance, height
    // //     double v = shootSpeed, h = Constants.SHOOTER_HEIGHT, g = 9.81;
    // //     double v2 = v*v, v4 = v2*v2, d2 = d*d;

    // //     double discerminal = v4 - g * (g*d2 + 2*h*v2); 

    // //     if (discerminal < 0) {
    // //         return Double.NaN;
    // //     }

    // //     double tanTheta = v2 - Math.sqrt(discerminal) / g*d; // - or + ?????
    // //     return Math.atan(tanTheta);
    // // }

    // public void resetEncoder() {
    //     encoder.setPosition(0);
    // }

    // public void stopElevator() {
    //     elevatorMotor.stopMotor();
    // }

    @Override
    public void periodic() {
        // if (getPos() >= Constants.Elevator_pos_limit || getPos() <= 0) {
        //     stopElevator();
        //     // System.out.println("elevator up to limit");
        // }
        // SmartDashboard.putNumber("Hood/Current Angle", encoder.getPosition());
    }
}