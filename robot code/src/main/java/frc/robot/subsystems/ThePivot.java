package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.StaticConstants;

public class ThePivot extends SubsystemBase {
    private boolean brakeEnabled;
    private boolean softLimitEnabled;
    private TalonFX motor;
    private double positionCoefficient = 1.0/116.666666667;
    private ProfiledPIDController profiledPIDController;
    private ArmFeedforward armFeedforward;
    private TrapezoidProfile.Constraints lowerConstraints = new TrapezoidProfile.Constraints(50, 30);
    private TrapezoidProfile.Constraints raiseConstraints = new TrapezoidProfile.Constraints(300, 300);
    private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(StaticConstants.ThePivot.encoderID);

    //TODO: set as a rio constant
    private double encoderZero = 0.8896;
    
    public ThePivot(){
        motor = new TalonFX(StaticConstants.ThePivot.ID);
        motor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(StaticConstants.ThePivot.forwardLimit / positionCoefficient)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(StaticConstants.ThePivot.reverseLimit / positionCoefficient));
        motor.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(12)
        .withPeakReverseVoltage(-3));
        brakeEnabled = true;
        softLimitEnabled = true;
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(true);
        motor.setPosition(0);
        motor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(StaticConstants.ThePivot.supplyCurrentLimit));
        motor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(StaticConstants.ThePivot.statorCurrentLimit));

        //TODO: set them in constants
        profiledPIDController = new ProfiledPIDController(2, 4, 0, lowerConstraints);
        armFeedforward = new ArmFeedforward(0, 0.45, 0, 0);
        profiledPIDController.setTolerance(0.003 / positionCoefficient);
        //profiledPIDController.setIntegratorRange(-10, 10);
        profiledPIDController.setIZone(0.01 / positionCoefficient);

        SmartDashboard.putData(profiledPIDController);
    }
    public void setDutyCycle(double d){
        motor.set(d);
    }
    public double getEncoderPosition(){
        double pos = 1 - absoluteEncoder.getAbsolutePosition() - encoderZero;
        if(pos < 0) pos++;
        //return Math.abs(pos - getPosition()) < 0.05 ? pos : getPosition();
        return pos;
    }
    public Command runVoltage(double voltage) {
        return runEnd(() -> {
            motor.setVoltage(voltage);
        }, () -> {
            motor.set(0);
        });
    }
    public void setPosition(double position){
        Supplier<?> intakePositionObj = SubsystemIO.getInstance().getValue("Intake Pivot: Position");
        TrapezoidProfile.Constraints constraints = position < getPosition() ? lowerConstraints : raiseConstraints;
        TrapezoidProfile.State state = new TrapezoidProfile.State(position / positionCoefficient, 0);
        double output = profiledPIDController.calculate(motor.getPosition().getValueAsDouble(), state, constraints)
        + armFeedforward.calculate(
            Math.PI * 2 * (motor.getPosition().getValueAsDouble() * positionCoefficient - DynamicConstants.ThePivot.uprightPosition + 0.25),
            Math.PI * 2 * (motor.getVelocity().getValueAsDouble() * positionCoefficient),
            Math.PI * 2 * (motor.getAcceleration().getValueAsDouble() * positionCoefficient))
        // - Math.cos(
        //     Math.PI * 2 * (
        //             (intakePositionObj != null && intakePositionObj.get() instanceof Double ? (Double) intakePositionObj.get() : 0.0)
        //             - DynamicConstants.Intake.pivotUprightPosition
        //         )
        //     ) * DynamicConstants.ThePivot.secondSegmentFeedforwardConstant
        ;
        motor.setVoltage(output);
        SmartDashboard.putNumber("ThePivot goal", profiledPIDController.getSetpoint().position * positionCoefficient);
        SmartDashboard.putNumber("ThePivot Position Error", profiledPIDController.getPositionError() * positionCoefficient);
    }
    public double getPosition(){
        return motor.getPosition().getValueAsDouble() * positionCoefficient;
    }
    public double getVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }
    public boolean atSetpoint(){
        return profiledPIDController.atGoal();
    }
    public boolean belowVelocityThreshold(){
        return Math.abs(motor.getVelocity().getValueAsDouble()) < DynamicConstants.ThePivot.shootVelocityThreshold;
    }
    public Command setPositionCommand(DoubleSupplier position, boolean dontEnd){
        return runOnce(() -> {resetProfiledPIDController();}).andThen(runEnd(() -> {
            setPosition(position.getAsDouble());
        },
        () -> {
            motor.set(0);
        }).until(() -> profiledPIDController.atGoal() && !dontEnd));
    }
    public void resetProfiledPIDController(){
        profiledPIDController.reset(motor.getPosition().getValueAsDouble(), motor.getVelocity().getValueAsDouble());
    }


    public Command switchNeutralMode() {
        return runOnce(() -> {
        if (brakeEnabled == true) {
            motor.setNeutralMode(NeutralModeValue.Coast);
            brakeEnabled = false;
        } 
        else if (brakeEnabled == false) {
            motor.setNeutralMode(NeutralModeValue.Brake);
            brakeEnabled = true;
        }
        });
    }

    public Command switchSoftLimit() {
        return runOnce(() -> {
        if (softLimitEnabled == true) {
            motor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false));
            softLimitEnabled = false;
        }
        else if (softLimitEnabled == false) {
            motor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
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
        motor.getConfigurator().setPosition(0);
        });
    }
    @Override
    public void periodic(){
        if(getEncoderPosition() < 0.01 && DriverStation.isDisabled()){ 
            motor.setPosition(0);
        }
    }
}
