package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ThePivot;

public class AmpSetpoint extends SequentialCommandGroup{
    public AmpSetpoint(IntakePivot intakePivot, IntakeWheels intakeWheels, ThePivot thePivot){
        addCommands(
            intakePivot.setPositionCommand(() -> DynamicConstants.Intake.pivotAmpPosition)
            .alongWith(thePivot.setPositionCommand(() -> DynamicConstants.ThePivot.ampPosition))
        );
    }
}
