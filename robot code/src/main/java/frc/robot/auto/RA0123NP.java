package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntegratedShooterCommand;
import frc.robot.commands.ResetHeadingOnTag;
import frc.robot.commands.ResetPose;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ThePivot;
import frc.robot.util.AutoChooser;

public class RA0123NP extends SequentialCommandGroup{
    public RA0123NP(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll, PhotonVision pv){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory RA1R = AutoChooser.openTrajectoryFile("RA1R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory R2BR = AutoChooser.openTrajectoryFile("RB2R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory RB3R = AutoChooser.openTrajectoryFile("RB3R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        addCommands(
            new ResetPose(drivetrainSubsystem, ll, RA1R.getInitialTargetHolonomicPose()),
            new ResetHeadingOnTag(drivetrainSubsystem, pv, -60.0),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2),
            new DriveAtPath(drivetrainSubsystem, RA1R, ll, false, pv).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3000), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2).unless(() -> !intakeWheels.hasPiece()),
            new DriveAtPath(drivetrainSubsystem, R2BR, ll, false, pv).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3000), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2).unless(() -> !intakeWheels.hasPiece()),
            new DriveAtPath(drivetrainSubsystem, RB3R, ll, false, pv).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3000), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2).unless(() -> !intakeWheels.hasPiece())
        );
    }
}
