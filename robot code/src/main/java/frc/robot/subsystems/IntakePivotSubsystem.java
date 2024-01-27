package frc.robot.subsystems;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {
    private TalonFX pivotMotor;
    private ShuffleboardTab tab; 
    private double max;
    private GenericEntry speed; 
    private boolean stop;
    public IntakePivotSubsystem() {
        pivotMotor = new TalonFX(22);
        tab = Shuffleboard.getTab("Pivot");
        speed = tab.add("Max Speed", 0.5).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
        max = speed.getDouble(0.5);
        stop = false;
    }
    public void periodic() {
         max = speed.getDouble(0.5);
    }
    public Command pivot() {
        return runEnd(() -> {
            if (!stop) {
              pivotMotor.set(max);
            } else {
                pivotMotor.set(0);
            }
        }, () -> {
            pivotMotor.set(0);
        });
    }

    public Command outtake() {
        return runEnd(() -> {
            pivotMotor.set(-0.25);
        },() -> {
            pivotMotor.set(0);
        });
    } 
}
