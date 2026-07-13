package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Speed.*;
import static frc.robot.Constants.MotorID.*;

public class IntakeSub extends SubsystemBase {
    private final SparkMax arm = new SparkMax(armID,  MotorType.kBrushless);
    private final TalonFX roller = new TalonFX(rollerID);

    public IntakeSub(){
        resetArmPos();

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } 
 
    /* Roller */
    public void rollSuck() {
        roller.set(rollerSpeed);
    }

    public void rollSpit() {
        roller.set(-rollerSpeed);
    }

    public void stopRoller() {
        roller.stopMotor();
    }

    /* Arm */
    public void up() {
        arm.set(armTeleopSpeed);
    }

    public void down() {
        arm.set(-0.2); // armTeleopSpeed
    }

    public void setArmSpeed(double speed) {
        arm.set(speed);
    } 

    public void stopArm() {
        arm.stopMotor();
    }

    public double getArmPos() {
        return arm.getEncoder().getPosition();
    }

    public void resetArmPos() {
        arm.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm pos", getArmPos());
    }
}