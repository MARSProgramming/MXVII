package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ThePivot;

public class IntakeCommand extends SequentialCommandGroup{
    public IntakeCommand(IntakePivot intakePivot, IntakeWheels intakeWheels, ThePivot mThePivot){
        // addCommands(
        //     intakeWheels.intakeCommand().deadlineWith(
        //         intakePivot.setPositionCommand(() -> DynamicConstants.Intake.pivotIntakePosition, true),
        //         mThePivot.setPositionCommand(() -> DynamicConstants.ThePivot.zeroPosition, false)
        //     ),
        //     intakePivot.zeroIntake()
        // );
        addCommands(intakeWheels.intakeCommand().alongWith(
                intakePivot.setPositionCommand(() -> DynamicConstants.Intake.pivotIntakePosition, true),
                mThePivot.setPositionCommand(() -> DynamicConstants.ThePivot.zeroPosition, false)
            ).until(() -> intakeWheels.hasPiece()),
            intakePivot.zeroIntake()
        );
    }
}
