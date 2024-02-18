package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.ThePivot;


public class ResetPositions extends SequentialCommandGroup {
    IntakePivot mIntakePivot;
    ThePivot mThePivot;
    Climber mClimber;
    
    public ResetPositions(IntakePivot mIntakePivot, ThePivot mThePivot, Climber mClimber) {
       addCommands(
            mIntakePivot.zeroPosition(),
            mThePivot.zeroPosition(),
            mClimber.zeroPosition()
       );
    }
}
