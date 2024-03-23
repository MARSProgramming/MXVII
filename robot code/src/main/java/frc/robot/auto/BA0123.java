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
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ThePivot;
import frc.robot.util.AutoChooser;

public class BA0123 extends SequentialCommandGroup{
    public BA0123(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll, PhotonVision pv){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory BA1 = AutoChooser.openTrajectoryFile("BA1", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory B1B = AutoChooser.openTrajectoryFile("B1B", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory BB2 = AutoChooser.openTrajectoryFile("BB2", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory B23 = AutoChooser.openTrajectoryFile("B23", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        addCommands(
            new ResetPose(drivetrainSubsystem, ll, BA1.getInitialTargetHolonomicPose()),
            drivetrainSubsystem.zeroGyroscope(-121.5),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            (new DriveAtPath(drivetrainSubsystem, BA1, ll, false, pv).andThen(new DriveAtPath(drivetrainSubsystem, B1B, ll, false, pv))).alongWith(new IntakeCommand(intakePivot, intakeWheels, thePivot))
            .deadlineWith(shooterFlywheel.runVelocity(() -> DynamicConstants.ShooterFlywheel.idleVelocity)).withTimeout(6),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            new DriveAtPath(drivetrainSubsystem, BB2, ll, false, pv).alongWith(new IntakeCommand(intakePivot, intakeWheels, thePivot)).deadlineWith(shooterFlywheel.runVelocity(() -> DynamicConstants.ShooterFlywheel.idleVelocity))
            .withTimeout(3),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            new DriveAtPath(drivetrainSubsystem, B23, ll, false, pv).alongWith(new IntakeCommand(intakePivot, intakeWheels, thePivot)).deadlineWith(shooterFlywheel.runVelocity(() -> DynamicConstants.ShooterFlywheel.idleVelocity))
            .withTimeout(3),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3)
        );
    }
}
