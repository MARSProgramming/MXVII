package frc.robot.subsystems;
import java.util.Map;

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
    private TalonFX master;
    private TalonFX follower;
    private ShuffleboardTab tab;
    private GenericEntry inputVoltage;
    private GenericEntry secondaryInputVoltage;
    private double max; 
    private double opposingMax;

    public Climber() {
        master = new TalonFX(StaticConstants.Climber.masterID);
        follower = new TalonFX(StaticConstants.Climber.followID);
        master.getConfigurator().apply(new TalonFXConfiguration());
        follower.getConfigurator().apply(new TalonFXConfiguration());
        master.setNeutralMode(NeutralModeValue.Brake);
        follower.setNeutralMode(NeutralModeValue.Brake);
        master.setInverted(false);

        tab = Shuffleboard.getTab("Climber");
        inputVoltage = tab.add("Voltage", 0.5).getEntry();
        secondaryInputVoltage = tab.add("Reverse Voltage", 0.5).getEntry();
        max = inputVoltage.getDouble(0.5);
        opposingMax = inputVoltage.getDouble(-0.5);
        follower.setControl(new Follower(StaticConstants.ShooterFlywheel.masterID, false));
    }

    public void periodic() {
        max = inputVoltage.getDouble(0.5);
        opposingMax = secondaryInputVoltage.getDouble(-0.5);
    }
    public Command runVoltage() {
        return runEnd(() -> {
            master.setVoltage(max);
        },
        () -> {
            master.setVoltage(0);
        });
    }

    public Command runVoltageSecondary() {
        return runEnd(() -> {
            master.setVoltage(opposingMax);
        },
        () -> {
            master.setVoltage(0);
        });
    }
    public double getVoltage(){
        return master.getMotorVoltage().getValueAsDouble();
    }
    public Command runVoltageCommand(double voltage){
        return runEnd(() -> {
            master.setVoltage(voltage);
        },
        () -> {
            master.setVoltage(0);
        });
    }

}
