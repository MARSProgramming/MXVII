package frc.robot.subsystems;
import java.util.Map;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
    

    public Climber() {
        left = new TalonFX(StaticConstants.Climber.leftID);
        right = new TalonFX(StaticConstants.Climber.rightID);
        left.getConfigurator().apply(new TalonFXConfiguration());
        right.getConfigurator().apply(new TalonFXConfiguration());
        left.setNeutralMode(NeutralModeValue.Brake);
        right.setNeutralMode(NeutralModeValue.Brake);

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
    public Command runVoltage() {
        return runEnd(() -> {
            left.setVoltage(max);
            right.setVoltage(max);
        },
        () -> {
            left.setVoltage(0);
            right.setVoltage(0);
        });
    }
    public Command runVoltageNegative() {
        return runEnd(() -> {
            left.setVoltage(opposingMax);
            right.setVoltage(opposingMax);
        },
        () -> {
            left.setVoltage(0);
            right.setVoltage(0);
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
  /*
    public Command runVoltageCommand(double voltage){
        return runEnd(() -> {
            left.setVoltage(voltage);
        },
        () -> {
            right.setVoltage(0);
        });
    } */

}
