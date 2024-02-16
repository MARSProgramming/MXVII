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
    private boolean brakeEnabled;
    private boolean softLimitEnabled;
    public IntakePivot() {
        pivotMotor = new TalonFX(StaticConstants.IntakePivot.ID);
        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        pivotMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(StaticConstants.IntakePivot.forwardLimit / positionCoefficient)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(StaticConstants.IntakePivot.reverseLimit / positionCoefficient));
        pivotMotor.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(6)
        .withPeakReverseVoltage(-6));
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        boolean brakeEnabled = true;
        pivotMotor.setInverted(true);
        pivotMotor.setPosition(0);
        profiledPIDController = new ProfiledPIDController(0.35, 0, 0, new TrapezoidProfile.Constraints(120, 80));
        armFeedforward = new ArmFeedforward(0, 0.4, 0, 0);
        profiledPIDController.setTolerance(0.01);
    }
    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            double v = voltage;
            if(v > 1){v = 1;}
            if(v < -1){v = -1;}
            pivotMotor.setVoltage(v);
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

    public Command switchNeutralMode() {
        return runOnce(() -> {
        if (brakeEnabled == true) {
            pivotMotor.setNeutralMode(NeutralModeValue.Coast);
            brakeEnabled = false;
        } 

        if (brakeEnabled == false) {
            pivotMotor.setNeutralMode(NeutralModeValue.Brake);
            brakeEnabled = true;
        }
        });
    }

    public Command switchSoftLimit() {
        return runOnce(() -> {
        if (softLimitEnabled == true) {
            pivotMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false));
        }
        if (softLimitEnabled == false) {
            pivotMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(StaticConstants.IntakePivot.forwardLimit / positionCoefficient)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(StaticConstants.IntakePivot.reverseLimit / positionCoefficient));
        }
     }); 
    }

    public Command zeroPosition() {
        return runOnce(() -> {
        pivotMotor.getConfigurator().setPosition(0);
        });
    }
}
