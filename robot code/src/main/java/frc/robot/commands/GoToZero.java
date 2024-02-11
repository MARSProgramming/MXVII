package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.ThePivot;

public class GoToZero extends ParallelCommandGroup{
    public GoToZero(IntakePivot intakePivot, ThePivot thePivot){
        addCommands(
            intakePivot.zeroIntake(),
            thePivot.setPositionCommand(() -> DynamicConstants.ThePivot.zeroPosition)
        );
    }
}
