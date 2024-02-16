package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    public ThePivot(){
        motor = new TalonFX(StaticConstants.ThePivot.ID);
        motor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(StaticConstants.ThePivot.forwardLimit / positionCoefficient)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(StaticConstants.ThePivot.reverseLimit / positionCoefficient));
        motor.getConfigurator().apply(new VoltageConfigs()
        .withPeakForwardVoltage(2)
        .withPeakReverseVoltage(-2));
        brakeEnabled = true;
        softLimitEnabled = true;
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(true);
        motor.setPosition(0);

        profiledPIDController = new ProfiledPIDController(0.4, 0, 0, new TrapezoidProfile.Constraints(100, 100));
        armFeedforward = new ArmFeedforward(0, 0, 0, 0);
        profiledPIDController.setTolerance(0.005);

        SmartDashboard.putData(profiledPIDController);
    }
    public void setDutyCycle(double d){
        motor.set(d);
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
        double output = profiledPIDController.calculate(motor.getPosition().getValueAsDouble(), position / positionCoefficient)
        - armFeedforward.calculate(
            Math.PI * 2 * (motor.getPosition().getValueAsDouble() * positionCoefficient - DynamicConstants.ThePivot.uprightPosition - 0.25),
            Math.PI * 2 * (motor.getVelocity().getValueAsDouble() * positionCoefficient),
            Math.PI * 2 * (motor.getAcceleration().getValueAsDouble() * positionCoefficient))
        - Math.cos(
            Math.PI * 2 * (
                    (intakePositionObj != null && intakePositionObj.get() instanceof Double ? (Double) intakePositionObj.get() : 0.0)
                    - DynamicConstants.Intake.pivotUprightPosition
                )
            ) * DynamicConstants.ThePivot.secondSegmentFeedforwardConstant;
        motor.setVoltage(output);
    }
    public double getPosition(){
        return motor.getPosition().getValueAsDouble() * positionCoefficient;
    }
    public double getVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }
    public Command setPositionCommand(DoubleSupplier position){
        return runEnd(() -> {
            setPosition(position.getAsDouble());
        },
        () -> {
            resetProfiledPIDController();
            motor.set(0);
        }).until(() -> profiledPIDController.atGoal());
    }
    public void resetProfiledPIDController(){
        profiledPIDController.reset(motor.getPosition().getValueAsDouble());
    }


    public Command switchNeutralMode() {
        return runOnce(() -> {
        if (brakeEnabled == true) {
            motor.setNeutralMode(NeutralModeValue.Coast);
            brakeEnabled = false;
        } 

        if (brakeEnabled == false) {
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
        }
        if (softLimitEnabled == false) {
            motor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(StaticConstants.IntakePivot.forwardLimit / positionCoefficient)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(StaticConstants.IntakePivot.reverseLimit / positionCoefficient));
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
        
    }
}
