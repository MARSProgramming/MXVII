package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.ThePivot;

public class GoToZero extends ParallelCommandGroup{
    public GoToZero(IntakePivot intakePivot, ThePivot thePivot){
        addCommands(
            //TODO: change to actually read the pivot position
            new WaitCommand(0.5).andThen(intakePivot.zeroIntake()),
            thePivot.setPositionCommand(() -> DynamicConstants.ThePivot.zeroPosition, false)
        );
    }
}
