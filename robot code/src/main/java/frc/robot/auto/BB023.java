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

public class BB023 extends SequentialCommandGroup{
    public BB023(DrivetrainSubsystem drivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll, PhotonVision pv){
        addRequirements(thePivot, shooterFlywheel, intakePivot, intakeWheels, drivetrainSubsystem);

        PathPlannerTrajectory BB1R = AutoChooser.openTrajectoryFile("BB1R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory BB2R = AutoChooser.openTrajectoryFile("BB2R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory BB3R = AutoChooser.openTrajectoryFile("BB3R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        PathPlannerTrajectory BB6R = AutoChooser.openTrajectoryFile("BB6R", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        
        PathPlannerTrajectory BBL = AutoChooser.openTrajectoryFile("BBL", drivetrainSubsystem, drivetrainSubsystem.getPigeonAngle());
        addCommands(
            new ResetPose(drivetrainSubsystem, ll, BB1R.getInitialTargetHolonomicPose()),
            new ResetHeadingOnTag(drivetrainSubsystem, pv, 180.0),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2),
            //new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2).unless(() -> !intakeWheels.hasPiece()),
            new DriveAtPath(drivetrainSubsystem, BB2R, ll, false, pv).until(() -> drivetrainSubsystem.getPose().getX() < 1.5 && intakeWheels.hasPiece()).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3500), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            //new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2).unless(() -> !intakeWheels.hasPiece()),
            intakeWheels.outtake().alongWith(shooterFlywheel.runVelocity(() -> 4000)).withTimeout(0.3),
            new DriveAtPath(drivetrainSubsystem, BB3R, ll, false, pv).until(() -> drivetrainSubsystem.getPose().getX() < 1.5 && intakeWheels.hasPiece()).deadlineWith(new IntakeCommand(intakePivot, intakeWheels, thePivot).withTimeout(2).andThen(intakePivot.setPositionCommand(() -> 0, false).alongWith(shooterFlywheel.runVelocity(() -> 3500), intakeWheels.runVoltage(3).unless(() -> intakeWheels.hasPiece())))),
            //new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2).unless(() -> !intakeWheels.hasPiece()),
            new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(3),
            new WaitCommand(4),
            new DriveAtPath(drivetrainSubsystem, BBL, ll, false, pv)
            //new IntegratedShooterCommand(intakeWheels, shooterFlywheel, thePivot, drivetrainSubsystem).withTimeout(2).unless(() -> !intakeWheels.hasPiece())
        );
    }
}
