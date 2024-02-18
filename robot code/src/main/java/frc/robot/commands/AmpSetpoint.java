package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ThePivot;

public class AmpSetpoint extends SequentialCommandGroup{
    IntakePivot mIntakePivot;
    IntakeWheels mIntakeWheels;
    ThePivot mThePivot;
    public AmpSetpoint(IntakePivot intakePivot, IntakeWheels intakeWheels, ThePivot thePivot){
        mIntakePivot = intakePivot;
        mThePivot = thePivot;
        addRequirements(thePivot, intakePivot);
        addCommands(
            intakePivot.setPositionCommand(() -> DynamicConstants.Intake.pivotAmpPosition, false),
            thePivot.setPositionCommand(() -> DynamicConstants.ThePivot.ampPosition, true)
        );
    }

}
