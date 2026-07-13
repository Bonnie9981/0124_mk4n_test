package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

public class ArmPosCmd extends Command {
    private final IntakeSub in;
    private final boolean armDown;

    private final PIDController armPIDgo = new PIDController(0.01, 0, 0);
    private final PIDController armPIDback = new PIDController(0.05, 0, 0); // TODO: armPID

    public ArmPosCmd(IntakeSub in, boolean armDown){
        this.in = in;
        this.armDown = armDown;
        addRequirements(in);
    }

    @Override
    public void execute(){
        if (armDown) {
            in.resetArmPos();
            in.setArmSpeed(armPIDgo.calculate(in.getArmPos(), -5));
        }
        else  in.setArmSpeed(armPIDback.calculate(in.getArmPos(), 0));
    }

    @Override
    public void end(boolean interrupted){
        in.setArmSpeed(0);
    }
        
    @Override
    public boolean isFinished(){
        return armPIDgo.atSetpoint() || armPIDback.atSetpoint();
    }
}