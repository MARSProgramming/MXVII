package frc.robot.subsystems;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.StaticConstants;

public class Climber extends SubsystemBase {
    private TalonFX left;
    private TalonFX right;
    private ShuffleboardTab tab;
    private GenericEntry inputVoltage;
    private GenericEntry secondaryInputVoltage;
    private double max; 
    private double opposingMax;
    private double positionCoefficient;
    private boolean softLimitEnabled;
    

    public Climber() {
        left = new TalonFX(StaticConstants.Climber.leftID);
        right = new TalonFX(StaticConstants.Climber.rightID);

        left.getConfigurator().apply(new TalonFXConfiguration());
        right.getConfigurator().apply(new TalonFXConfiguration());
        left.setNeutralMode(NeutralModeValue.Brake);
        right.setNeutralMode(NeutralModeValue.Brake);
        softLimitEnabled = true;
        // where are the limits

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

        left.setInverted(true);

        tab = Shuffleboard.getTab("Climber");
        inputVoltage = tab.add("Voltage", 1).getEntry();
        secondaryInputVoltage = tab.add("Reverse Voltage", -1).getEntry();
        max = inputVoltage.getDouble(1);
        opposingMax = inputVoltage.getDouble(-1);
        positionCoefficient = 1.0/16.0;
    }

    public void periodic() {
        max = inputVoltage.getDouble(1);
        opposingMax = secondaryInputVoltage.getDouble(-1);
        
    }
    public Command runVoltage(int voltage) {
        return runEnd(() -> {
            left.setVoltage(voltage);
            right.setVoltage(voltage);
        },
        () -> {
            left.setVoltage(0);
            right.setVoltage(0);
        });
    }
    public Command runVoltageLeft() {
        return runEnd(() -> {
            left.setVoltage(max);
        },
        () -> {
            left.set(0);
        });
    }
    public Command runVoltageRight() {
        return runEnd(() -> {
            right.setVoltage(max);
        },
        () -> {
            right.set(0);
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


    public Command switchSoftLimit() {
        return runOnce(() -> {
        if (softLimitEnabled == true) {
            left.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false));
            right.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false));
        }
        if (softLimitEnabled == false) {
            // WE DO NOT HAVE ANY CLIMBER POSITION LIMITS IN STATIC CONSTANTS
            left.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(0)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0));
            right.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(0)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0));
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
