package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.StaticConstants;

public class IntakePivot extends SubsystemBase {
    private TalonFX pivotMotor;
    private double positionCoefficient = 1.0/16.0*15.0/24.0;
    private ProfiledPIDController profiledPIDController;
    private ArmFeedforward armFeedforward;
    private boolean brakeEnabled;
    private boolean softLimitEnabled;
    private DigitalInput reverseLimitSwitch;
    private boolean resetPos = false;
    public IntakePivot() {
        pivotMotor = new TalonFX(StaticConstants.IntakePivot.ID);
        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        pivotMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(StaticConstants.IntakePivot.forwardLimit / positionCoefficient)
        .withReverseSoftLimitEnable(false)
        .withReverseSoftLimitThreshold(StaticConstants.IntakePivot.reverseLimit / positionCoefficient));
        pivotMotor.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(10)
        .withPeakReverseVoltage(-10));
        pivotMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(StaticConstants.IntakePivot.supplyCurrentLimit).withStatorCurrentLimitEnable(true).withStatorCurrentLimit(StaticConstants.IntakePivot.statorCurrentLimit));
        pivotMotor.setNeutralMode(NeutralModeValue.Coast);
        brakeEnabled = false;
        softLimitEnabled = true;
        pivotMotor.setInverted(true);
        pivotMotor.setPosition(0);
        profiledPIDController = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(100,60));
        armFeedforward = new ArmFeedforward(0, 0.7, 0, 0);
        profiledPIDController.setTolerance(0.02 / positionCoefficient);

        reverseLimitSwitch = new DigitalInput(StaticConstants.IntakePivot.reverseLimitSwitchID);
    }
    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            double v = voltage;
            if(v > 1){v = 1;}
            if(v < -1){v = -1;}
            pivotMotor.setVoltage(v);
        },
        () -> {
            pivotMotor.set(0);
        });
    }
    public void setDutyCycle(double d){
        pivotMotor.set(d);
    }
    public void setVoltage(double d){
        pivotMotor.setVoltage(d);
    }
    public void setPosition(double position){
        double output = profiledPIDController.calculate(pivotMotor.getPosition().getValueAsDouble(), position / positionCoefficient)
        + armFeedforward.calculate(
            Math.PI * 2 * (pivotMotor.getPosition().getValueAsDouble() * positionCoefficient - DynamicConstants.Intake.pivotUprightPosition + 0.25),
            Math.PI * 2 * (pivotMotor.getVelocity().getValueAsDouble() * positionCoefficient),
            Math.PI * 2 * (pivotMotor.getAcceleration().getValueAsDouble() * positionCoefficient));
        pivotMotor.setVoltage(output);
    }


    public double getPosition(){
        return pivotMotor.getPosition().getValueAsDouble() * positionCoefficient;
    }
    public double getVoltage(){
        return pivotMotor.getMotorVoltage().getValueAsDouble();
    }
    public Command setPositionCommand(DoubleSupplier position, boolean dontEnd){
        return runOnce(() -> {resetProfiledPIDController();}).andThen(runEnd(() -> {
            setPosition(position.getAsDouble());
        },
        () -> {
            pivotMotor.set(0);
        })).until(() -> (profiledPIDController.atGoal() && !dontEnd));
    }

    public Command zeroIntake() {
        return runOnce(() -> {
            //pivotMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withReverseSoftLimitEnable(false).withReverseSoftLimitThreshold(-10000));
            resetProfiledPIDController();
            resetPos = true;
            i = 0;
        }).andThen(runEnd(() -> {
            if(getPosition() > DynamicConstants.Intake.pivotUprightPosition){
                pivotMotor.setVoltage(-6);
            }
            else{
                pivotMotor.setVoltage(Math.abs(getPosition()) * -20);
            }
        }, () -> {
            pivotMotor.set(0);
            //pivotMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withReverseSoftLimitEnable(true));
        }).until(() -> getLimitSwitch()));
    }

    public boolean getLimitSwitch(){
        return !reverseLimitSwitch.get();
    }
    public Command setPositionDontEndAtSetpointCommand(DoubleSupplier position){
        return runEnd(() -> {
            setPosition(position.getAsDouble());
        },
        () -> {
            resetProfiledPIDController();
            pivotMotor.set(0);
        });
    }
    public void resetProfiledPIDController(){
        profiledPIDController.reset(pivotMotor.getPosition().getValueAsDouble());
    }

    public Command switchNeutralMode() {
        return runOnce(() -> {
        if (brakeEnabled == true) {
            pivotMotor.setNeutralMode(NeutralModeValue.Coast);
            brakeEnabled = false;
        } 
        else if (brakeEnabled == false) {
            pivotMotor.setNeutralMode(NeutralModeValue.Brake);
            brakeEnabled = true;
        }
        });
    }

    public Command switchSoftLimit() {
        return runOnce(() -> {
        if (softLimitEnabled == true) {
            pivotMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false));
            softLimitEnabled = false;
        }
        else if (softLimitEnabled == false) {
            pivotMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(StaticConstants.IntakePivot.forwardLimit / positionCoefficient)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(StaticConstants.IntakePivot.reverseLimit / positionCoefficient));
            softLimitEnabled = true;
        }
     }); 
    }

    public Command zeroPosition() {
        return runOnce(() -> {
        pivotMotor.getConfigurator().setPosition(0);
        });
    }

    int i = 0;
    @Override
    public void periodic(){
        if(getLimitSwitch() && pivotMotor.getPosition().getValueAsDouble() != 0){
            pivotMotor.setPosition(0);
            if(i > 20){
                resetPos = false;
            }
            i++;
        }
    }
}
