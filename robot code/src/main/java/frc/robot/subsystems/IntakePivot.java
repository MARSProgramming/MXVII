package frc.robot.subsystems;
import java.lang.constant.DynamicConstantDesc;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
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
    private double positionCoefficient = 1.0/16.0;
    private ProfiledPIDController profiledPIDController;
    private ArmFeedforward armFeedforward;
    public IntakePivot() {
        pivotMotor = new TalonFX(StaticConstants.IntakePivot.ID);
        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        pivotMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(StaticConstants.IntakePivot.forwardLimit / positionCoefficient)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(StaticConstants.IntakePivot.reverseLimit / positionCoefficient));
        pivotMotor.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(4)
        .withPeakReverseVoltage(4));
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        pivotMotor.setInverted(false);
        profiledPIDController = new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(1, 1));
        armFeedforward = new ArmFeedforward(0, 0.1, 0, 0);
    }
    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            pivotMotor.setVoltage(voltage);
        },
        () -> {
            pivotMotor.setVoltage(0);
        });
    }
    public void setPosition(double position){
        double output = profiledPIDController.calculate(pivotMotor.getPosition().getValueAsDouble(), position / positionCoefficient)
        + armFeedforward.calculate(
            pivotMotor.getPosition().getValueAsDouble() * positionCoefficient - DynamicConstants.Intake.pivotUprightPosition,
            pivotMotor.getVelocity().getValueAsDouble() * positionCoefficient,
            pivotMotor.getAcceleration().getValueAsDouble() * positionCoefficient);
        pivotMotor.setVoltage(12 * (output));
    }
    public double getPosition(){
        return pivotMotor.getPosition().getValueAsDouble() * positionCoefficient;
    }
    public Command setPositionCommand(DoubleSupplier position){
        return runEnd(() -> {
            setPosition(position.getAsDouble());
        },
        () -> {
            pivotMotor.setVoltage(0);
        }).until(() -> pivotMotor.getClosedLoopError().getValueAsDouble() < 0.5);
    }
}
