package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntegratedShooterCommand;
import frc.robot.commands.ResetPose;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ThePivot;
import frc.robot.util.AutoChooser;

public class RA0123 extends SequentialCommandGroup{
    public RA0123(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory RA1 = AutoChooser.openTrajectoryFile("RA1", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory R1B = AutoChooser.openTrajectoryFile("R1B", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory RB2 = AutoChooser.openTrajectoryFile("RB2", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory R23 = AutoChooser.openTrajectoryFile("R23", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        addCommands(
            new ResetPose(drivetrainSubsystem, ll, RA1.getInitialTargetHolonomicPose()),
            drivetrainSubsystem.zeroGyroscope(-63.0),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            (new DriveAtPath(drivetrainSubsystem, RA1, ll, false).alongWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2))).andThen(new DriveAtPath(drivetrainSubsystem, R1B, ll, false).deadlineWith(intakePivot.setPositionCommand(() -> 0, false)))
            .deadlineWith(shooterFlywheel.runVelocity(() -> DynamicConstants.ShooterFlywheel.idleVelocity)).withTimeout(6),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3).unless(() -> !intakeWheels.hasPiece()),
            new DriveAtPath(drivetrainSubsystem, RB2, ll, false).alongWith(new IntakeCommand(intakePivot, intakeWheels, thePivot)).deadlineWith(shooterFlywheel.runVelocity(() -> DynamicConstants.ShooterFlywheel.idleVelocity))
            .withTimeout(3),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3).unless(() -> !intakeWheels.hasPiece()),
            new DriveAtPath(drivetrainSubsystem, R23, ll, false).alongWith(new IntakeCommand(intakePivot, intakeWheels, thePivot)).deadlineWith(shooterFlywheel.runVelocity(() -> DynamicConstants.ShooterFlywheel.idleVelocity))
            .withTimeout(3),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(5).unless(() -> !intakeWheels.hasPiece())
        );
    }
}
