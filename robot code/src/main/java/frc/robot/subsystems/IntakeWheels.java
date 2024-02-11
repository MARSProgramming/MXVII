package frc.robot.subsystems;
import java.time.DayOfWeek;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.StaticConstants;

public class IntakeWheels extends SubsystemBase {
    private TalonFX intakeMotor;

    //left and right relative to the robot's front
    private AnalogInput irSensorLeft;
    private AnalogInput irSensorRight;
    public IntakeWheels() {
        intakeMotor = new TalonFX(StaticConstants.IntakeWheels.ID);
        intakeMotor.getConfigurator().apply(new TalonFXConfiguration());
        intakeMotor.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(12)
        .withPeakReverseVoltage(12));
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setInverted(false);

        irSensorLeft = new AnalogInput(StaticConstants.IntakeWheels.leftIrID);
        irSensorLeft.setAverageBits(4);
        irSensorLeft.setOversampleBits(4);

        irSensorRight = new AnalogInput(StaticConstants.IntakeWheels.rightIrID);
        irSensorRight.setAverageBits(4);
        irSensorRight.setOversampleBits(4);
    }

    public double getIrReading(){
       return Math.max(irSensorLeft.getAverageVoltage(), irSensorRight.getAverageVoltage());
      
    }

    public void periodic() {
        SmartDashboard.putNumber("left sensor reading", irSensorLeft.getAverageVoltage());
        SmartDashboard.putNumber("right sensor reading", irSensorRight.getAverageVoltage());
        SmartDashboard.putBoolean("ir sensor boolean",  (getIrReading() > DynamicConstants.Intake.irSensorThreshold));
    }
    
    public void runDutyCycle(double dc) {
        intakeMotor.set(dc);
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
        }).until(() -> (getIrReading() > DynamicConstants.Intake.irSensorThreshold));
        
    }


}
