package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.StaticConstants;

public class ShooterFlywheel extends SubsystemBase {
    private TalonFX master;
    private TalonFX follower;
    
    public ShooterFlywheel() {
        master = new TalonFX(StaticConstants.ShooterFlywheel.masterID);
        follower = new TalonFX(StaticConstants.ShooterFlywheel.followID);
        master.getConfigurator().apply(new TalonFXConfiguration());
        follower.getConfigurator().apply(new TalonFXConfiguration());
        master.setNeutralMode(NeutralModeValue.Coast);
        follower.setNeutralMode(NeutralModeValue.Coast);
        master.setInverted(false);
        follower.setControl(new Follower(StaticConstants.ShooterFlywheel.masterID, true));
        Slot0Configs config = new Slot0Configs();
        config.kP = 0.0;
        config.kI = 0.0;
        config.kD = 0.0;
        config.kV = 0.5;
        master.getConfigurator().apply(config);
    }
    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            master.setVoltage(voltage);
        }, () -> {
            master.setVoltage(0);
        });
    }
    public void setVelocity(double velocity){
        master.setControl(new VelocityDutyCycle(velocity));
    }
    public Command runVelocity(double velocity) {
        return runEnd(() -> {
            setVelocity(velocity);
        }, () -> {
            master.setVoltage(0);
        });
    }
}
