package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntegratedShooterCommand;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ThePivot;
import frc.robot.util.AutoChooser;

public class RB0213 extends SequentialCommandGroup{
    public RB0213(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory RB3 = AutoChooser.openTrajectoryFile("RB3", drivetrainSubsystem, Rotation2d.fromDegrees(0));
        PathPlannerTrajectory R32 = AutoChooser.openTrajectoryFile("R32", drivetrainSubsystem, Rotation2d.fromDegrees(30.14));
        PathPlannerTrajectory R21 = AutoChooser.openTrajectoryFile("R21", drivetrainSubsystem, Rotation2d.fromDegrees(-16.06));
        PathPlannerTrajectory R14 = AutoChooser.openTrajectoryFile("R14", drivetrainSubsystem, Rotation2d.fromDegrees(-90));
        PathPlannerTrajectory R41 = AutoChooser.openTrajectoryFile("R41", drivetrainSubsystem, Rotation2d.fromDegrees(0));
        //PathPlannerTrajectory R3B = AutoChooser.openTrajectoryFile("R3B", drivetrainSubsystem, Rotation2d.fromDegrees(0));
        addCommands(
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem),
            new DriveAtPath(drivetrainSubsystem, RB3, ll).withTimeout(3).alongWith(new IntakeCommand(intakePivot, intakeWheels)),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).andThen(thePivot.setPositionCommand(() -> DynamicConstants.ThePivot.zeroPosition, false)).withTimeout(4),
            new DriveAtPath(drivetrainSubsystem, R32, ll).withTimeout(5).alongWith(new WaitCommand(0.5).andThen(new IntakeCommand(intakePivot, intakeWheels))),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).andThen(thePivot.setPositionCommand(() -> DynamicConstants.ThePivot.zeroPosition, false)).withTimeout(4),
            new DriveAtPath(drivetrainSubsystem, R21, ll).withTimeout(5).alongWith(new IntakeCommand(intakePivot, intakeWheels)),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).andThen(thePivot.setPositionCommand(() -> DynamicConstants.ThePivot.zeroPosition, false).withTimeout(4)),
            new DriveAtPath(drivetrainSubsystem, R14, ll).withTimeout(5).alongWith(new WaitCommand(1).andThen(new IntakeCommand(intakePivot, intakeWheels))),
            new DriveAtPath(drivetrainSubsystem, R41, ll).withTimeout(5).alongWith(new IntakeCommand(intakePivot, intakeWheels)),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem)
        );
    }
}
