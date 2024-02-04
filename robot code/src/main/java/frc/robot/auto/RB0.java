package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveAtPath;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ThePivot;
import frc.robot.util.AutoChooser;

public class RB0 extends SequentialCommandGroup{
    public RB0(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory RB2 = AutoChooser.openTrajectoryFile("RB2", drivetrainSubsystem, Rotation2d.fromDegrees(0));
        addCommands(
            shooterFlywheel.runVelocity(() -> 3000.0).alongWith(
                new WaitCommand(2).andThen(intakeWheels.runVoltage(-6.0))
            ).withTimeout(5),
            new DriveAtPath(drivetrainSubsystem, RB2)
        );
    }
}
