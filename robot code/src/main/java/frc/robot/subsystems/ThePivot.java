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
        .withPeakForwardVoltage(6)
        .withPeakReverseVoltage(-3));
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(true);
        motor.setPosition(0);

        profiledPIDController = new ProfiledPIDController(1.6, 0, 0, new TrapezoidProfile.Constraints(80, 50));
        armFeedforward = new ArmFeedforward(0, 0.45, 0, 0);
        profiledPIDController.setTolerance(0.0015 / positionCoefficient);

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
        // - Math.cos(
        //     Math.PI * 2 * (
        //             (intakePositionObj != null && intakePositionObj.get() instanceof Double ? (Double) intakePositionObj.get() : 0.0)
        //             - DynamicConstants.Intake.pivotUprightPosition
        //         )
        //     ) * DynamicConstants.ThePivot.secondSegmentFeedforwardConstant
        ;
        motor.setVoltage(output);
        SmartDashboard.putNumber("ThePivot Position Error", profiledPIDController.getPositionError());
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
        return motor.getVelocity().getValueAsDouble() < DynamicConstants.ThePivot.shootVelocityThreshold;
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
        profiledPIDController.reset(motor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic(){
        
    }
}
