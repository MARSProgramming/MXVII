package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.ThePivot;


public class SwitchNeutralModeCommand extends SequentialCommandGroup {
    IntakePivot mIntakePivot;
    ThePivot mThePivot;
    
    public SwitchNeutralModeCommand(IntakePivot mIntakePivot, ThePivot mThePivot) {
       addCommands(
            mIntakePivot.switchNeutralMode(),
            mThePivot.switchNeutralMode()
       );
    }
}
