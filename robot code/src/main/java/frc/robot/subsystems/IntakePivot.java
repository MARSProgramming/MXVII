package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.StaticConstants;

public class IntakePivot extends SubsystemBase {
    private TalonFX pivotMotor;
    private double positionCoefficient = 1.0/16.0*15.0/24.0;
    private ProfiledPIDController profiledPIDController;
    private ArmFeedforward armFeedforward;
    public IntakePivot() {
        pivotMotor = new TalonFX(StaticConstants.IntakePivot.ID);
        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        pivotMotor.setPosition(0);
        pivotMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(StaticConstants.IntakePivot.forwardLimit / positionCoefficient)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(StaticConstants.IntakePivot.reverseLimit / positionCoefficient));
        pivotMotor.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(1.1)
        .withPeakReverseVoltage(-1.1));
        pivotMotor.getConfigurator().apply(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.5));
        pivotMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.5));
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        pivotMotor.setInverted(false);
        profiledPIDController = new ProfiledPIDController(0.35, 0, 0, new TrapezoidProfile.Constraints(100, 100));
        armFeedforward = new ArmFeedforward(0, 0.4, 0, 0);
        profiledPIDController.setTolerance(0.01);
    }
    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            pivotMotor.setVoltage(voltage);
        },
        () -> {
            pivotMotor.set(0);
        });
    }
    public void setDutyCycle(double d){
        pivotMotor.set(d);
    }
    public void setPosition(double position){
        double output = profiledPIDController.calculate(pivotMotor.getPosition().getValueAsDouble(), position / positionCoefficient)
        - armFeedforward.calculate(
            Math.PI * 2 * (pivotMotor.getPosition().getValueAsDouble() * positionCoefficient - DynamicConstants.Intake.pivotUprightPosition - 0.25),
            Math.PI * 2 * (pivotMotor.getVelocity().getValueAsDouble() * positionCoefficient),
            Math.PI * 2 * (pivotMotor.getAcceleration().getValueAsDouble() * positionCoefficient));
        pivotMotor.setVoltage(output);
    }


    public double getPosition(){
        return pivotMotor.getPosition().getValueAsDouble() * positionCoefficient;
    }
    public double getVoltage(){
        return pivotMotor.getMotorVoltage().getValueAsDouble();
    }
    public Command setPositionCommand(DoubleSupplier position, boolean dontEnd){
        return runEnd(() -> {
            setPosition(position.getAsDouble());
        },
        () -> {
            resetProfiledPIDController();
            pivotMotor.set(0);
        }).until(() -> profiledPIDController.atGoal() && !dontEnd);
    }
    public Command setPositionDontEndAtSetpointCommand(DoubleSupplier position){
        return runEnd(() -> {
            setPosition(position.getAsDouble());
        },
        () -> {
            resetProfiledPIDController();
            pivotMotor.set(0);
        });
    }
    public void resetProfiledPIDController(){
        profiledPIDController.reset(pivotMotor.getPosition().getValueAsDouble());
    }
}
