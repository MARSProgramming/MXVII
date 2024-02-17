package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ThePivot;

public class InitializeTrapSetpoint extends SequentialCommandGroup{
    IntakePivot mIntakePivot;
    IntakeWheels mIntakeWheels;
    ThePivot mThePivot;
    
    public InitializeTrapSetpoint(IntakePivot intakePivot, ThePivot thePivot){
        mIntakePivot = intakePivot;
        mThePivot = thePivot;
        //addRequirements(thePivot, intakePivot);
        addRequirements(thePivot);
        addCommands(
            thePivot.setPositionCommand(() -> DynamicConstants.ThePivot.trapPosition, true)
            //intakePivot.setPositionCommand(() -> DynamicConstants.Intake.pivotTrapPosition, false)
        );
    }

}