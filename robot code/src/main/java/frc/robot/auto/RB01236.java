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

public class RB01236 extends SequentialCommandGroup{
    public RB01236(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll, PhotonVision pv){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory RB1R = AutoChooser.openTrajectoryFile("RB1R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory RB2R = AutoChooser.openTrajectoryFile("RB2R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory RB3R = AutoChooser.openTrajectoryFile("RB3R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory RB6R = AutoChooser.openTrajectoryFile("RB6R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        
        PathPlannerTrajectory RB8 = AutoChooser.openTrajectoryFile("RB8", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        addCommands(
            new ResetPose(drivetrainSubsystem, ll, RB1R.getInitialTargetHolonomicPose()),
            new ResetHeadingOnTag(drivetrainSubsystem, pv, 0.0),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2),
            new DriveAtPath(drivetrainSubsystem, RB1R, ll, false, pv).until(() -> drivetrainSubsystem.getPose().getX() > 14.7 && intakeWheels.hasPiece()).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3500), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            intakeWheels.outtake().alongWith(shooterFlywheel.runVelocity(() -> 4000)).withTimeout(0.3),
            //new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2).unless(() -> !intakeWheels.hasPiece()),
            new DriveAtPath(drivetrainSubsystem, RB2R, ll, false, pv).until(() -> drivetrainSubsystem.getPose().getX() > 14.7 && intakeWheels.hasPiece()).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3500), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            //new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2).unless(() -> !intakeWheels.hasPiece()),
            intakeWheels.outtake().alongWith(shooterFlywheel.runVelocity(() -> 4000)).withTimeout(0.3),
            new DriveAtPath(drivetrainSubsystem, RB3R, ll, false, pv).until(() -> drivetrainSubsystem.getPose().getX() > 15.2 && intakeWheels.hasPiece()).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3500), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            //new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2).unless(() -> !intakeWheels.hasPiece()),
            intakeWheels.outtake().alongWith(shooterFlywheel.runVelocity(() -> 4000)).withTimeout(0.5),
            new DriveAtPath(drivetrainSubsystem, RB6R, ll, true, pv, () -> intakeWheels.hasPiece()).until(() -> drivetrainSubsystem.getPose().getX() > 14.7 && intakeWheels.hasPiece()).deadlineWith((new IntakeCommand(intakePivot, intakeWheels, thePivot)).withTimeout(6).andThen(intakePivot.zeroIntake().alongWith(shooterFlywheel.runVelocity(() -> 3500), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            intakeWheels.outtake().alongWith(shooterFlywheel.runVelocity(() -> 4000)).withTimeout(0.5),
            new DriveAtPath(drivetrainSubsystem, RB8, ll, false, pv)
            //new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2).unless(() -> !intakeWheels.hasPiece())
        );
    }
}
