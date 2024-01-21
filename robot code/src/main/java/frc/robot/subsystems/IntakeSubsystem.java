package frc.robot.subsystems;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX intakeMotor;
    private ShuffleboardTab tab; 
    private double max;
    private GenericEntry speed; 
    private AnalogInput sensor;
    private boolean stop;
    public IntakeSubsystem() {
        intakeMotor = new TalonFX(12);
        sensor = new AnalogInput(0);
        tab = Shuffleboard.getTab("Intake");
        speed = tab.add("Max Speed", 0.5).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
        max = speed.getDouble(0.5);
        stop = false;
    }
    public void periodic() {
         max = speed.getDouble(0.5);
        SmartDashboard.putNumber("Sensor Voltage", sensor.getVoltage());

        if (sensor.getVoltage() >= 0.6) {
            stop = true;
        } else {
            stop = false;
        }

    }
    public Command intake() {
        return runEnd(() -> {
            if (!stop) {
              intakeMotor.set(max);
            } else {
                intakeMotor.set(0);
            }
        }, () -> {
            intakeMotor.set(0);
        });
    }

    public Command outtake() {
        return runEnd(() -> {
            intakeMotor.set(-0.25);
        },() -> {
            intakeMotor.set(0);
        });
    } 
}
