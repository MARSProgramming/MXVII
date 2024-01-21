package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX master;
    private TalonFX follower;
    private ShuffleboardTab tab;
    private double max;
    private GenericEntry speed;
    
    public ShooterSubsystem() {
        master = new TalonFX(23);
        follower = new TalonFX(13);
        tab = Shuffleboard.getTab("Shooter");
        follower.setControl(new Follower(master.getDeviceID(), false));
        speed = tab.add("Max Speed", 0.5).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    }

    public void periodic() {
        max = speed.getDouble(0.5);
    }

    public Command run() {
        return runEnd(() -> {
            master.set(max);
        }, () -> {
            master.set(0);
        });
    }
}
