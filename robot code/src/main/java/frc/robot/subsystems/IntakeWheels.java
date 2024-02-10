package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.StaticConstants;

public class IntakeWheels extends SubsystemBase {
    private TalonFX intakeMotor;
    private AnalogInput irSensor;

    private ShuffleboardTab tab; 
    private double max;
    private GenericEntry speed; 
    
    public IntakeWheels() {
        intakeMotor = new TalonFX(StaticConstants.IntakeWheels.ID);
        intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
        intakeMotor.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(12)
        .withPeakReverseVoltage(12));
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setInverted(true);

        irSensor = new AnalogInput(3);
        irSensor.setAverageBits(4);
        irSensor.setOversampleBits(4);
      //  tab = Shuffleboard.getTab("Integrated Shooter");

       // speed = tab.add("Intake Wheel Speed", 0.5).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
      //  max = speed.getDouble(0.5); 
    }

    public void periodic() {
   //     max = speed.getDouble(0.5);
    }

    public Command testRunIntake() {
        return runEnd(() -> {
            intakeMotor.set(max);
        }, () -> {
            intakeMotor.set(0);
        });
    }

    public double getIrReading(){
        return irSensor.getAverageVoltage();
    }

    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            intakeMotor.setVoltage(voltage);
        },
        () -> {
            intakeMotor.setVoltage(0);
        });
    }
    public void intake(){
        //TODO: make dynamic constant
        intakeMotor.setVoltage(8);
    }

    public Command outtake() {
        return runEnd(() -> {
            intakeMotor.setVoltage(-4);
        }, () -> {
            intakeMotor.setVoltage(0);
        });
    }

    
    public double getVoltage(){
        return intakeMotor.getMotorVoltage().getValueAsDouble();
    }
    public Command intakeCommand(){
        return runEnd(() -> {
            intake();
        },
        () -> {
            intakeMotor.setVoltage(0);
        }).until(() -> getIrReading() > DynamicConstants.Intake.irSensorThreshold);
    }
}
