package frc.robot.subsystems;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.StaticConstants;

public class IntakePivotSubsystem extends SubsystemBase {
    private TalonFX pivotMotor;
    public IntakePivotSubsystem() {
        pivotMotor = new TalonFX(StaticConstants.IntakePivot.ID);
        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        pivotMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(StaticConstants.IntakePivot.forwardLimit)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(StaticConstants.IntakePivot.reverseLimit));
        pivotMotor.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(2)
        .withPeakReverseVoltage(2));
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        pivotMotor.setInverted(false);
    }
    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            pivotMotor.setVoltage(voltage);
        },
        () -> {
            pivotMotor.setVoltage(0);
        });
    }
}
