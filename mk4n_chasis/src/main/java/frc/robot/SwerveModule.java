package frc.robot;


import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;


public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;


    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;


    // Configured feedforward for drive motor (voltage = kS + kV * velocity + kA * acceleration)
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(
        Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA
    );


    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0); // Open loop: outputs a fixed value without feedback correction
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0); // Closed loop: adjusts output based on sensor feedback


    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);


    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;


        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);


        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();


        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setPosition(0.0);
    }


    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        SwerveModuleState current = getState();
   
        @SuppressWarnings("deprecation") // WPILib 2025 3
        SwerveModuleState optimized = SwerveModuleState.optimize(
            desiredState,
            current.angle
        );
   
        desiredState = optimized;
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Open loop control: uses a fixed duty cycle for the drive motor
        if (isOpenLoop) {
            double output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveDutyCycle.Output = output;
            // Deadband
            if (Math.abs(output) <= 0.05) {
                mDriveMotor.setControl(new DutyCycleOut(0)); // safer Kraken set
            } else {
                mDriveMotor.setControl(driveDutyCycle);
            }


        } // Closed loop control: uses velocity and feedforward for the drive motor
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }


    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }


    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }


    // Get Module State (speed in m/s, angle in radians)
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference),
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }


    // Get Module Position (distance in meters, angle in radians)
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference),
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    public double getDriveSupplyCurrent() {
        return mDriveMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getAngleSupplyCurrent() {
        return mAngleMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getDriveStatorCurrent() {
        return mDriveMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getAngleStatorCurrent() {
        return mAngleMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getDriveVoltage() {
        return mDriveMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getAngleVoltage() {
        return mDriveMotor.getMotorVoltage().getValueAsDouble();
    }
}