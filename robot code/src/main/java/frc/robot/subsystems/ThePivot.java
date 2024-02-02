package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
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

public class ThePivot extends SubsystemBase {
    private TalonFX motor;
    private double positionCoefficient = 1.0/116.666666667;
    private ProfiledPIDController profiledPIDController;
    private ArmFeedforward armFeedforward;
    public ThePivot(){
        motor = new TalonFX(StaticConstants.ThePivot.ID);
        motor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(StaticConstants.ThePivot.forwardLimit / positionCoefficient)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(StaticConstants.ThePivot.reverseLimit / positionCoefficient));
        motor.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(3)
        .withPeakReverseVoltage(-3));
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(true);
        motor.setPosition(0);

        profiledPIDController = new ProfiledPIDController(0.0, 0, 0, new TrapezoidProfile.Constraints(1, 1));
        armFeedforward = new ArmFeedforward(0, 0.9, 0, 0);
    }
    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            motor.setVoltage(voltage);
        }, () -> {
            motor.set(0);
        });
    }
    public void setPosition(double position){
        Object intakePositionObj = SubsystemIO.getInstance().getValue("Intake: Pivot Position").get();
        double output = profiledPIDController.calculate(motor.getPosition().getValueAsDouble(), position / positionCoefficient)
        + armFeedforward.calculate(
            motor.getPosition().getValueAsDouble() * positionCoefficient - DynamicConstants.ThePivot.uprightPosition,
            motor.getVelocity().getValueAsDouble() * positionCoefficient,
            motor.getAcceleration().getValueAsDouble() * positionCoefficient)
        + Math.cos(
            Math.PI * 2 * (
                    (intakePositionObj instanceof Double ? (Double) intakePositionObj : 0.0)
                    - DynamicConstants.Intake.pivotUprightPosition
                )
            ) * DynamicConstants.ThePivot.secondSegmentFeedforwardConstant;
        motor.setVoltage(output);
    }
    public double getPosition(){
        return motor.getPosition().getValueAsDouble() * positionCoefficient;
    }
    public double getVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }
    public Command setPositionCommand(DoubleSupplier position){
        return runEnd(() -> {
            setPosition(position.getAsDouble());
        },
        () -> {
            motor.setVoltage(0);
        }).until(() -> profiledPIDController.getPositionError() < 0.5);
    }

    @Override
    public void periodic(){
        
    }
}
