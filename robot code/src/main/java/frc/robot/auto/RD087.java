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

public class RD087 extends SequentialCommandGroup{
    public RD087(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory RD8 = AutoChooser.openTrajectoryFile("RD8", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory R8F = AutoChooser.openTrajectoryFile("R8F", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory RF7 = AutoChooser.openTrajectoryFile("RF7", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory R7F = AutoChooser.openTrajectoryFile("R7F", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        addCommands(
            new ResetPose(drivetrainSubsystem, ll, RD8.getInitialTargetHolonomicPose()),
            drivetrainSubsystem.zeroGyroscope(57),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            new DriveAtPath(drivetrainSubsystem, RD8, ll, true).deadlineWith(new WaitCommand(1).andThen(new IntakeCommand(intakePivot, intakeWheels, thePivot))).andThen(new DriveAtPath(drivetrainSubsystem, R8F, ll, false).deadlineWith(shooterFlywheel.runVelocity(() -> DynamicConstants.ShooterFlywheel.idleVelocity), intakePivot.setPositionCommand(() -> 0, true))),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            new DriveAtPath(drivetrainSubsystem, RF7, ll, true).deadlineWith(new WaitCommand(1.5).andThen(new IntakeCommand(intakePivot, intakeWheels, thePivot))).andThen(new DriveAtPath(drivetrainSubsystem, R7F, ll, false).deadlineWith(shooterFlywheel.runVelocity(() -> DynamicConstants.ShooterFlywheel.idleVelocity), intakePivot.setPositionCommand(() -> 0, true))),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3)
        );
    }
}
