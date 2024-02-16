package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.ThePivot;

public class GoToZero extends SequentialCommandGroup{
    public GoToZero(IntakePivot intakePivot, ThePivot thePivot){
        addCommands(
            intakePivot.zeroIntake(),
            thePivot.setPositionCommand(() -> DynamicConstants.ThePivot.zeroPosition, false)
        );
    }
}
