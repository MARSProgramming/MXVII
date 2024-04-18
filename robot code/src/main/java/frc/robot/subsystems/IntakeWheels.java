package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import frc.robot.util.Rumble;

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
        intakeMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(StaticConstants.IntakeWheels.supplyCurrentLimit));
        intakeMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(StaticConstants.IntakeWheels.statorCurrentLimit));

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
    private int i = 0;
    public boolean hasPiece(){
        if(getIrReading() > DynamicConstants.Intake.irSensorThresholdShoot){
            i++;
        }
        else{
            i = 0;
        }
        return i >= 5;
        //return getIrReading() > DynamicConstants.Intake.irSensorThreshold;
    }

    public void periodic() {
        SmartDashboard.putNumber("left sensor reading", irSensorLeft.getAverageVoltage());
        SmartDashboard.putNumber("right sensor reading", irSensorRight.getAverageVoltage());
        SmartDashboard.putBoolean("ir sensor boolean",  (getIrReading() > DynamicConstants.Intake.irSensorThresholdShoot));
    }
    
    public void setDutyCycle(double dc) {
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
    public Command runVoltageUntilIRReading(double voltage, double IR) {
        return runEnd(() -> {
            if(getIrReading() > IR){
                intakeMotor.setVoltage(voltage);
            }
            else{
                intakeMotor.setVoltage(0);
            }
        },
        () -> {
            intakeMotor.setVoltage(0);
        });
    }
    public void intake(){
        intakeMotor.setVoltage(DynamicConstants.Intake.intakeVoltage);
    }
    public void setVoltage(double d){
        intakeMotor.setVoltage(d);
    }

    public Command outtake() {
        return runEnd(() -> {
            intakeMotor.setVoltage(DynamicConstants.Intake.outtakeVoltage);
        }, () -> {
            intakeMotor.setVoltage(0);
        });
    }

    
    public double getVoltage(){
        return intakeMotor.getMotorVoltage().getValueAsDouble();
    }
    private int j = 0;
    // public boolean intakeDetectedPiece(){
    //     if(getIrReading() > DynamicConstants.Intake.irSensorThresholdIntake){
    //         j++;
    //     }
    //     else{
    //         j = 0;
    //     }
    //     return j >= 1;
    // }
    public Command intakeCommand(){
        return runEnd(() -> {
            if(getIrReading() < DynamicConstants.Intake.irSensorThresholdIntake){
                intake();
            }
            else{
                intakeMotor.setVoltage(0);
            }
        },
        () -> {
            intakeMotor.setVoltage(0);
            Rumble.getInstance().rumbleController1(1);
        });
        
    }
    // public Command intakeCommand(){
    //     return runEnd(() -> {
    //         intake();
    //     },
    //     () -> {
    //         intakeMotor.setVoltage(0);
    //         Rumble.getInstance().rumbleController1(1);
    //     }).until(() -> (getIrReading() > DynamicConstants.Intake.irSensorThresholdIntake));
    // }


}
