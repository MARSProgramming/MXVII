package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveAtPath;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntegratedShooterCommand;
import frc.robot.commands.ResetHeadingOnTag;
import frc.robot.commands.ResetPose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ThePivot;
import frc.robot.util.AutoChooser;

public class RA014 extends SequentialCommandGroup{
    public RA014(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll, PhotonVision pv){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory RA1 = AutoChooser.openTrajectoryFile("RA1", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory R14 = AutoChooser.openTrajectoryFile("R14", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory R4G = AutoChooser.openTrajectoryFile("R4G", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        addCommands(
            new ResetPose(drivetrainSubsystem, ll, RA1.getInitialTargetHolonomicPose()),
            new ResetHeadingOnTag(drivetrainSubsystem, pv, -60.0),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            new DriveAtPath(drivetrainSubsystem, RA1, ll, false, pv).alongWith(new IntakeCommand(intakePivot, intakeWheels, thePivot)).withTimeout(3),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            new DriveAtPath(drivetrainSubsystem, R14, ll, true, pv).deadlineWith(new WaitCommand(0.5), new IntakeCommand(intakePivot, intakeWheels, thePivot)).withTimeout(4),
            new DriveAtPath(drivetrainSubsystem, R4G, ll, false, pv).deadlineWith(intakePivot.setPositionCommand(() -> 0, true)),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3)
        );
    }
}
