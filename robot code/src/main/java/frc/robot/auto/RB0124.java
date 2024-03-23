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

public class RB0124 extends SequentialCommandGroup{
    public RB0124(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll, PhotonVision pv){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory RB1R = AutoChooser.openTrajectoryFile("RB1R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory RB2 = AutoChooser.openTrajectoryFile("RB2", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory R24 = AutoChooser.openTrajectoryFile("R24", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory R4G = AutoChooser.openTrajectoryFile("R4G", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        addCommands(
            new ResetPose(drivetrainSubsystem, ll, RB1R.getInitialTargetHolonomicPose()),
            new ResetHeadingOnTag(drivetrainSubsystem, pv, 0.0),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2),
            new DriveAtPath(drivetrainSubsystem, RB1R, ll, true, pv).withTimeout(3.5).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3000), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3).unless(() -> !intakeWheels.hasPiece()),
            new DriveAtPath(drivetrainSubsystem, RB2, ll, true, pv).alongWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(3)).deadlineWith(shooterFlywheel.runVelocity(() -> 3000)),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3).unless(() -> !intakeWheels.hasPiece()),
            new DriveAtPath(drivetrainSubsystem, R24, ll, true, pv).andThen(new DriveAtPath(drivetrainSubsystem, R4G, ll, false, pv).withTimeout(4)).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(6).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3000), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3).unless(() -> !intakeWheels.hasPiece())
        );
    }
}
