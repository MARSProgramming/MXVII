package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ThePivot;
import frc.robot.util.AutoChooser;

public class RB0213 extends SequentialCommandGroup{
    public RB0213(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory RB2 = AutoChooser.openTrajectoryFile("RB2", drivetrainSubsystem, Rotation2d.fromDegrees(0));
        PathPlannerTrajectory R2B = AutoChooser.openTrajectoryFile("R2B", drivetrainSubsystem, Rotation2d.fromDegrees(0));
        PathPlannerTrajectory RB1 = AutoChooser.openTrajectoryFile("RB1", drivetrainSubsystem, Rotation2d.fromDegrees(0));
        PathPlannerTrajectory R1B = AutoChooser.openTrajectoryFile("R1B", drivetrainSubsystem, Rotation2d.fromDegrees(0));
        PathPlannerTrajectory RB3 = AutoChooser.openTrajectoryFile("RB3", drivetrainSubsystem, Rotation2d.fromDegrees(0));
        PathPlannerTrajectory R3B = AutoChooser.openTrajectoryFile("R3B", drivetrainSubsystem, Rotation2d.fromDegrees(0));
        addCommands(
            shooterFlywheel.runVelocity(() -> 3000.0).alongWith(
                new WaitCommand(0.5).andThen(intakeWheels.runVoltage(-6.0))
            ).withTimeout(1),
            new DriveAtPath(drivetrainSubsystem, RB2).withTimeout(3).andThen(new DriveAtPath(drivetrainSubsystem, R2B).withTimeout(3)).deadlineWith(new IntakeCommand(intakePivot, intakeWheels)),
            shooterFlywheel.runVelocity(() -> 3000.0).alongWith(
                new WaitCommand(0.5).andThen(intakeWheels.runVoltage(-6.0))
            ).withTimeout(1),
            new DriveAtPath(drivetrainSubsystem, RB1).withTimeout(3).andThen(new DriveAtPath(drivetrainSubsystem, R1B).withTimeout(3)).deadlineWith(new IntakeCommand(intakePivot, intakeWheels)),
            shooterFlywheel.runVelocity(() -> 3000.0).alongWith(
                new WaitCommand(0.5).andThen(intakeWheels.runVoltage(-6.0))
            ).withTimeout(1),
            new DriveAtPath(drivetrainSubsystem, RB3).withTimeout(3).andThen(new DriveAtPath(drivetrainSubsystem, R3B).withTimeout(3)).deadlineWith(new IntakeCommand(intakePivot, intakeWheels)),
            shooterFlywheel.runVelocity(() -> 3000.0).alongWith(
                new WaitCommand(0.5).andThen(intakeWheels.runVoltage(-6.0))
            ).withTimeout(1)
        );
    }
}
