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
        pilotControls.put("Start", "Close Shuttle");
        pilotControls.put("Back Button", "Runs Intake Wheels");
        pilotControls.put("Right Trigger", "Shoot");
        pilotControls.put("Left Trigger", "Intake");
        pilotControls.put("Right Bumper", "Score Amp");
        pilotControls.put("Left Bumper", "Intake From Source");
        pilotControls.put("X", "Allign To Tag");
        pilotControls.put("Y", "Zero Yaw");
        pilotControls.put("A", "Far Shuttle");
        pilotControls.put("B", "Drive Robot Centric");
        pilotControls.put("D-pad Up", "The Pivot Down");
        pilotControls.put("D-pad Down", "The Pivot Up");
        pilotControls.put("D-pad Left", "IntakePivot Retract");
        pilotControls.put("D-pad Right", "IntakePivot Extend");
        Map<String, String> copilotControls = new HashMap<>();
        copilotControls.put("Left Trigger", "Amp Setpoint");
        copilotControls.put("Right Trigger", "Shoot Manual");
        copilotControls.put("Left Bumper", "Go to Zero");
        copilotControls.put("Right Bumper", "Outake Trap");
        copilotControls.put("X", "Deploy Climber arms");
        copilotControls.put("Y", "Trap Setpoint");
        copilotControls.put("A", "Climb Setpoint");
        copilotControls.put("B", "Lower Climbing Arms");
        copilotControls.put("Start", "Left Climber Arm Override Limit");
        copilotControls.put("Back Button", "Right Climber Arm Override Limit");
        copilotControls.put("D-pad Up", "Climbers Up");
        copilotControls.put("D-pad Down", "Climbers Down");
        copilotControls.put("D-pad Left", "Intake Extend");
        copilotControls.put("D-pad Right", "Intake Retract");
        

        displayedValues.put("Intake Wheels: IR sensor", () -> intakeWheels.getIrReading());
        displayedValues.put("Intake Wheels: Has Piece", () -> intakeWheels.hasPiece());
        displayedValues.put("Intake Wheels: Voltage", () -> intakeWheels.getVoltage());
        displayedValues.put("Intake Pivot: Position", () -> intakePivot.getPosition());
        displayedValues.put("Intake Pivot: Voltage", () -> intakePivot.getVoltage());
        displayedValues.put("Intake Pivot: Limit Switch", () -> intakePivot.getLimitSwitch());
        displayedValues.put("The Pivot: Integrated Position", () -> thePivot.getPosition());
        displayedValues.put("The Pivot: Encoder Position", () -> thePivot.getEncoderPosition());
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
