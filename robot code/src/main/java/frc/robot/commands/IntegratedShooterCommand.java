package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ShooterFlywheel;

public class IntegratedShooterCommand extends SequentialCommandGroup {
    public IntegratedShooterCommand(IntakeWheels intakeWheels, ShooterFlywheel shooterFlywheel) {
        addCommands(
            shooterFlywheel.runVelocity(() -> DynamicConstants.ShooterFlywheel.testVelocity),
            intakeWheels.outtake().onlyIf(() -> shooterFlywheel.atSpeed())
        );
    }
}
