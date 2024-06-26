package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.StaticConstants;

public class Climber extends SubsystemBase {
    private TalonFX left;
    private TalonFX right;
    private double positionCoefficient = 1.0/16.0;
    private boolean softLimitEnabled;
    private DigitalInput leftLimitSwitch;
    private DigitalInput rightLimitSwitch;
    

    public Climber() {
        left = new TalonFX(StaticConstants.Climber.leftID);
        right = new TalonFX(StaticConstants.Climber.rightID);

        left.getConfigurator().apply(new TalonFXConfiguration());
        right.getConfigurator().apply(new TalonFXConfiguration());
        left.setNeutralMode(NeutralModeValue.Brake);
        right.setNeutralMode(NeutralModeValue.Brake);
        softLimitEnabled = true;
        Slot0Configs rightconfig = new Slot0Configs();
        Slot0Configs leftconfig = new Slot0Configs();

        rightconfig.kP = StaticConstants.Climber.rightkP;
        rightconfig.kI = StaticConstants.Climber.rightkI;
        rightconfig.kD = StaticConstants.Climber.rightkD;

        leftconfig.kP = StaticConstants.Climber.leftkP;
        leftconfig.kI = StaticConstants.Climber.leftkI;
        leftconfig.kD = StaticConstants.Climber.leftkD;

        left.setInverted(true);
        right.getConfigurator().apply(rightconfig);
        left.getConfigurator().apply(leftconfig);


        left.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(StaticConstants.Climber.leftForwardLimit / positionCoefficient)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(StaticConstants.Climber.leftReverseLimit / positionCoefficient));
        right.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(StaticConstants.Climber.rightForwardLimit / positionCoefficient)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(StaticConstants.Climber.rightReverseLimit / positionCoefficient));

        left.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(20).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(60));

        right.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(20).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(60));
        
        leftLimitSwitch = new DigitalInput(StaticConstants.Climber.leftLimitSwitchID);
        rightLimitSwitch = new DigitalInput(StaticConstants.Climber.rightLimitSwitchID);

        left.setPosition(0);
        right.setPosition(0);
    }

    public void setRightPosition(double position){
        right.setControl(new PositionDutyCycle(position / positionCoefficient));
    }

    public Command setRightPositionCommand(double position){
        return runEnd(() -> {
            setRightPosition(position);
        }, () -> {
            right.set(0);
        });
    }

    public void setLeftPosition(double position){
        left.setControl(new PositionDutyCycle(position / positionCoefficient));
    }
    public Command setLeftPositionCommand(double position){
        return runEnd(() -> {
            setLeftPosition(position);
        }, () -> {
            left.set(0);
        });
    }

    public Command setPositionCommand(DoubleSupplier position) {
        return runEnd(() -> {
            setLeftPosition(position.getAsDouble());
            setRightPosition(position.getAsDouble());
        }, () -> {
            left.set(0);
            right.set(0);
        });
    }

    public Command climbToLimit() {
        return runEnd(() -> {
            setLeftVoltage(-5, false);
            setRightVoltage(-5, false);
        }, () -> {
            left.set(0);
            right.set(0);
        }).until(() -> (getLeftLimitSwitch() && getRightLimitSwitch()));
    }

    public void periodic() {
        
    }
    
    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            setLeftVoltage(voltage, false);
            setRightVoltage(voltage, false);
        },
        () -> {
            left.setVoltage(0);
            right.setVoltage(0);
        });
    }
    public Command runOneSideVoltage(double voltage, boolean leftSide){
        return runEnd(() -> {
            if(leftSide) setLeftVoltage(voltage, false);
            else setRightVoltage(voltage, false);
        },
        () -> {
            if(leftSide) setLeftVoltage(0, false);
            else setRightVoltage(0, false);
        });
    }
    public Command runOneSideVoltageOverrideLimit(double voltage, boolean leftSide){
        return runEnd(() -> {
            if(leftSide) setLeftVoltage(voltage, true);
            else setRightVoltage(voltage, true);
        },
        () -> {
            if(leftSide) left.set(0);
            else right.set(0);
        });
    }
    public double getLeftVoltage(){
        return left.getMotorVoltage().getValueAsDouble();
    }

    public double getRightVoltage(){
        return right.getMotorVoltage().getValueAsDouble();
    }

    public double getRightPosition() {
        return right.getPosition().getValueAsDouble() * positionCoefficient;
    }
 
    public double getLeftPosition() {
        return left.getPosition().getValueAsDouble() * positionCoefficient;
    }
    public boolean getLeftLimitSwitch(){
        return !leftLimitSwitch.get();
    }
    public boolean getRightLimitSwitch(){
        return !rightLimitSwitch.get();
    }
    public void setLeftVoltage(double v, boolean limitOverride){
        if(!getLeftLimitSwitch() || limitOverride || v > 0){
            left.setVoltage(v);
        }
    }
    public void setRightVoltage(double v, boolean limitOverride){
        if(!getRightLimitSwitch() || limitOverride || v > 0){
            right.setVoltage(v);
        }
    }
    public Command switchSoftLimit() {
        return runOnce(() -> {
        if (softLimitEnabled == true) {
            left.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false));
            right.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false));

            softLimitEnabled = false;
        }
        else if (softLimitEnabled == false) {
            left.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(StaticConstants.Climber.leftForwardLimit / positionCoefficient)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(StaticConstants.Climber.leftReverseLimit / positionCoefficient));
            right.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(StaticConstants.Climber.rightForwardLimit / positionCoefficient)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(StaticConstants.Climber.rightReverseLimit / positionCoefficient));

            softLimitEnabled = true;
        }
        });
    }

    public Command zeroPosition() {
        return runOnce(() -> {
            left.getConfigurator().setPosition(0);
            right.getConfigurator().setPosition(0);
        });
    }
}
