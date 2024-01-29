package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.StaticConstants;

public class IntakeWheels extends SubsystemBase {
    private TalonFX intakeMotor;
    private AnalogInput irSensor;
    public IntakeWheels() {
        intakeMotor = new TalonFX(StaticConstants.IntakeWheels.ID);
        intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
        intakeMotor.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(12)
        .withPeakReverseVoltage(12));
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setInverted(false);

        irSensor = new AnalogInput(0);
        irSensor.setAverageBits(4);
        irSensor.setOversampleBits(4);
    }

    public double getIrReading(){
        return irSensor.getAverageValue();
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
        intakeMotor.setVoltage(6);
    }
    public Command intakeCommand(){
        return runEnd(() -> {
            intake();
        },
        () -> {
            intakeMotor.setVoltage(0);
        }).until(() -> irSensor.getAverageValue() > DynamicConstants.Intake.irSensorThreshold);
    }
}
