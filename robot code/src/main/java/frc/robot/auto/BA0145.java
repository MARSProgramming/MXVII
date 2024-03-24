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
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ThePivot;
import frc.robot.util.AutoChooser;

public class BA0145 extends SequentialCommandGroup{
    public BA0145(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll, PhotonVision pv){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        //TODO: not fast enough
        PathPlannerTrajectory BA1 = AutoChooser.openTrajectoryFile("BA1", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory B14 = AutoChooser.openTrajectoryFile("B14", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory B4G = AutoChooser.openTrajectoryFile("B4G", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory BG5 = AutoChooser.openTrajectoryFile("BG5", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory B5G = AutoChooser.openTrajectoryFile("B5G", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        addCommands(
            new ResetPose(drivetrainSubsystem, ll, BA1.getInitialTargetHolonomicPose()),
            new ResetHeadingOnTag(drivetrainSubsystem, pv, -121.5),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            new DriveAtPath(drivetrainSubsystem, BA1, ll, false, pv).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot)),
            intakePivot.zeroIntake().andThen(new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3).unless(() -> !intakeWheels.hasPiece())),
            new DriveAtPath(drivetrainSubsystem, B14, ll, true, pv).deadlineWith(new WaitCommand(1), new IntakeCommand(intakePivot, intakeWheels, thePivot)).withTimeout(5),
            new DriveAtPath(drivetrainSubsystem, B4G, ll, false, pv).deadlineWith(intakePivot.setPositionCommand(() -> 0, true)).until(() -> drivetrainSubsystem.getPose().getX() < 3 && intakeWheels.hasPiece()),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            new DriveAtPath(drivetrainSubsystem, BG5, ll, true, pv).deadlineWith(new WaitCommand(1), new IntakeCommand(intakePivot, intakeWheels, thePivot)).withTimeout(5),
            new DriveAtPath(drivetrainSubsystem, B5G, ll, false, pv).deadlineWith(intakePivot.setPositionCommand(() -> 0, true)).until(() -> drivetrainSubsystem.getPose().getX() < 3 && intakeWheels.hasPiece()),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3)
        );
    }
}
