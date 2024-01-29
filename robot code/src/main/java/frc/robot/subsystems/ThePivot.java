package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.StaticConstants;

public class ThePivot extends SubsystemBase {
    private TalonFX motor;
    private double positionCoefficient = 1.0/16.0;
    public ThePivot(){
        motor = new TalonFX(StaticConstants.ThePivot.ID);
        motor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(StaticConstants.ThePivot.forwardLimit / positionCoefficient)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(StaticConstants.ThePivot.reverseLimit / positionCoefficient));
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(false);
        Slot0Configs config = new Slot0Configs();
        config.kP = 0.01;
        config.kI = 0.0;
        config.kD = 0.0;
        config.kG = 5;
        motor.getConfigurator().apply(config);
    }
    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            motor.setVoltage(voltage);
        }, () -> {
            motor.setVoltage(0);
        });
    }
    public void setPosition(double position){
        motor.setControl(new PositionDutyCycle(position / positionCoefficient));
    }
    public double getPosition(){
        return motor.getPosition().getValueAsDouble() * positionCoefficient;
    }
    public Command setPositionCommand(DoubleSupplier position){
        return runEnd(() -> {
            setPosition(position.getAsDouble());
        },
        () -> {
            motor.setVoltage(0);
        }).until(() -> motor.getClosedLoopError().getValueAsDouble() < 0.5);
    }
}
