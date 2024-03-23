package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntegratedShooterCommand;
import frc.robot.commands.ResetPose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ThePivot;
import frc.robot.util.AutoChooser;

public class BA0123NP extends SequentialCommandGroup{
    public BA0123NP(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll, PhotonVision pv){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory BA1R = AutoChooser.openTrajectoryFile("BA1R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory B2BR = AutoChooser.openTrajectoryFile("BB2R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory BB3R = AutoChooser.openTrajectoryFile("BB3R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        addCommands(
            new ResetPose(drivetrainSubsystem, ll, BA1R.getInitialTargetHolonomicPose()),
            drivetrainSubsystem.zeroGyroscope(-121.5),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(4),
            new DriveAtPath(drivetrainSubsystem, BA1R, ll, false, pv).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3000), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(4).unless(() -> !intakeWheels.hasPiece()),
            new DriveAtPath(drivetrainSubsystem, B2BR, ll, false, pv).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3000), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(4).unless(() -> !intakeWheels.hasPiece()),
            new DriveAtPath(drivetrainSubsystem, BB3R, ll, false, pv).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3000), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(4).unless(() -> !intakeWheels.hasPiece())
        );
    }
}
