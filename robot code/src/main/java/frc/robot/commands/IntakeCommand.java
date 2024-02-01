package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;

public class IntakeCommand extends SequentialCommandGroup{
    public IntakeCommand(IntakePivot intakePivot, IntakeWheels intakeWheels){
        addCommands(
            intakeWheels.intakeCommand().deadlineWith(
            intakePivot.setPositionCommand(() -> DynamicConstants.Intake.pivotIntakePosition)),
            intakePivot.setPositionCommand(() -> DynamicConstants.Intake.pivotStowPosition)
        );
    }
}
