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

public class RA012 extends SequentialCommandGroup{
    public RA012(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll, PhotonVision pv){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory RA1 = AutoChooser.openTrajectoryFile("RA1", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory RB2 = AutoChooser.openTrajectoryFile("RB2", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory R1B = AutoChooser.openTrajectoryFile("R1B", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        addCommands(
            new ResetPose(drivetrainSubsystem, ll, RA1.getInitialTargetHolonomicPose()),
            drivetrainSubsystem.zeroGyroscope(-63.0).withTimeout(0.1),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            new DriveAtPath(drivetrainSubsystem, RA1, ll, false, pv).andThen(new DriveAtPath(drivetrainSubsystem, R1B, ll, false, pv)).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot))
            .withTimeout(6),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            new DriveAtPath(drivetrainSubsystem, RB2, ll, false, pv).alongWith(new IntakeCommand(intakePivot, intakeWheels, thePivot)),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3)
        );
    }
}
