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
    
    public SubsystemIO(IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot){
        mInstance = this;

        Map<String, String> pilotControls = new HashMap<>();
        pilotControls.put("Right Trigger", "Shoot");
        pilotControls.put("X", "Amp");
        Map<String, String> copilotControls = new HashMap<>();
        copilotControls.put("Left Trigger", "test");
        Map<String, Supplier<?>> displayedValues = new HashMap<>();
        displayedValues.put("Intake: IR sensor", () -> intakeWheels.getIrReading());
        displayedValues.put("Intake: Pivot Position", () -> intakePivot.getPosition());
        dashboard = new BasicDash(pilotControls, copilotControls, displayedValues);
    }

    @Override
    public void periodic(){
        dashboard.periodic();
    }
}
