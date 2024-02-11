package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.StaticConstants;

public class ShooterFlywheel extends SubsystemBase {
    private TalonFX master;
    private TalonFX follower;

    private ShuffleboardTab tab; 
    private double max;
    private GenericEntry speed; 
    
    private double RPStoRPM = 60.0;
    public ShooterFlywheel() {
        master = new TalonFX(StaticConstants.ShooterFlywheel.masterID);
        follower = new TalonFX(StaticConstants.ShooterFlywheel.followID);
        master.getConfigurator().apply(new TalonFXConfiguration());
        follower.getConfigurator().apply(new TalonFXConfiguration());
        master.setNeutralMode(NeutralModeValue.Coast);
        follower.setNeutralMode(NeutralModeValue.Coast);
        master.setInverted(false);
        follower.setControl(new Follower(StaticConstants.ShooterFlywheel.masterID, false));
        Slot0Configs config = new Slot0Configs();
        config.kP = 0.0;
        config.kI = 0.0;
        config.kD = 0.0;
        config.kV = 0.0093;
        master.getConfigurator().apply(config);
    }

    public void periodic() {
        max = speed.getDouble(3000);
    }


    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            master.setVoltage(voltage);
        }, () -> {
            master.set(0);
        });
    }
    public void setVelocity(double velocity){
        master.setControl(new VelocityDutyCycle(velocity / RPStoRPM));
    }
    public double getVelocity(){
        return master.getVelocity().getValueAsDouble() * RPStoRPM;
    }

    public boolean atSpeed() {
        if (getVelocity() >= (DynamicConstants.ShooterFlywheel.testVelocity - 200)) {
            return true;
        } else {
            return false;
        }
    }


    public Command runVelocity(DoubleSupplier velocity) {
        return runEnd(() -> {
            setVelocity(velocity.getAsDouble());
        }, () -> {
            master.set(0);
        });
    }
}
