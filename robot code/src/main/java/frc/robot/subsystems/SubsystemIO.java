package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BasicDash;

public class SubsystemIO extends SubsystemBase{
    /*
     * Singleton structure
     * (see https://www.tutorialspoint.com/design_pattern/singleton_pattern.htm)
     */
    private static SubsystemIO mInstance;
    public static SubsystemIO getInstance(){
        if(mInstance == null) return null;
        return mInstance;
    }

    private BasicDash dashboard;
    /*
     * Creates a dashboard displaying the pilot/copilot controls and subsystem information
     */
    /*private IntakeWheels mIntakeWheels;
    private IntakePivot mIntakePivot;
    private ShooterFlywheel mShooterFlywheel;
    private ThePivot mThePivot;*/
    private Map<String, Supplier<?>> displayedValues = new HashMap<>();
    public SubsystemIO(DrivetrainSubsystem dt, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Climber climber){
        mInstance = this;
        /*mIntakeWheels = intakeWheels;
        mIntakePivot = intakePivot;
        mShooterFlywheel = shooterFlywheel;
        mThePivot = thePivot;*/

        Map<String, String> pilotControls = new HashMap<>();
        pilotControls.put("Right Trigger", "Shoot");
        pilotControls.put("X", "Amp");
        Map<String, String> copilotControls = new HashMap<>();
        copilotControls.put("Left Trigger", "test");

        displayedValues.put("Intake Wheels: IR sensor", () -> intakeWheels.getIrReading());
        displayedValues.put("Intake Wheels: Voltage", () -> intakeWheels.getVoltage());
        displayedValues.put("Intake Pivot: Position", () -> intakePivot.getPosition());
        displayedValues.put("Intake Pivot: Voltage", () -> intakePivot.getVoltage());
        displayedValues.put("Intake Pivot: Limit Switch", () -> intakePivot.getLimitSwitch());
        displayedValues.put("The Pivot: Position", () -> thePivot.getPosition());
        displayedValues.put("The Pivot: Voltage", () -> thePivot.getVoltage());
        displayedValues.put("Shooter: Velocity(RPM)", () -> shooterFlywheel.getVelocity());
        displayedValues.put("Drivetrain: X Position", () -> dt.getPose().getX());
        displayedValues.put("Drivetrain: Y Position", () -> dt.getPose().getY());
        displayedValues.put("Drivetrain: Rotation", () -> dt.getPigeonAngle());
        displayedValues.put("Left Climber Voltage", () -> climber.getLeftVoltage());
        displayedValues.put("Right Climber Voltage", () -> climber.getLeftVoltage());
        displayedValues.put("Left Climber Position", () -> climber.getLeftPosition());
        displayedValues.put("Right Climber Position", () -> climber.getRightPosition());
        displayedValues.put("Left Climber Limit Switch", () -> climber.getLeftLimitSwitch());
        displayedValues.put("Right Climber Limit Switch", () -> climber.getRightLimitSwitch());

        dashboard = new BasicDash(pilotControls, copilotControls, displayedValues);
    }

    @Override
    public void periodic(){
        dashboard.periodic();
    }

    public Supplier<?> getValue(String key){
        return displayedValues.get(key);
    }
}
