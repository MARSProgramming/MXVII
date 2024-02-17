package frc.robot.util;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.DoNothing;
import frc.robot.auto.RB0;
import frc.robot.auto.RB0321;
import frc.robot.auto.RDL;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ThePivot;

public class AutoChooser {
    private ShuffleboardTab preMatch;
    private SendableChooser<Command> autoChooser = new SendableChooser<>();
    private DrivetrainSubsystem drivetrainSubsystem;

    public AutoChooser(DrivetrainSubsystem mDrivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll){
        preMatch = Shuffleboard.getTab("Match");
        drivetrainSubsystem = mDrivetrainSubsystem;

        //auto plays
        autoChooser.setDefaultOption("Do Nothing", new DoNothing());
        autoChooser.addOption("RB0", new RB0(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RDL", new RDL(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RB0321", new RB0321(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));

        preMatch.add("Auto Play", autoChooser).withSize(2, 1).withPosition(4, 5);
    }

    public Command getSelected(){
        return autoChooser.getSelected();
    }

    public static PathPlannerTrajectory openTrajectoryFile(String name, DrivetrainSubsystem drivetrainSubsystem){
        PathPlannerTrajectory t = PathPlannerPath.fromPathFile(name).getTrajectory(drivetrainSubsystem.getChassisSpeeds(), Rotation2d.fromRadians(drivetrainSubsystem.getPigeonAngle()));
        return t;
    }
    public PathPlannerTrajectory openTrajectoryFileForAlliance(String name, DriverStation.Alliance alliance){
        //TODO: add thing to flip the path
        PathPlannerTrajectory t = PathPlannerPath.fromPathFile(name).getTrajectory(drivetrainSubsystem.getChassisSpeeds(), new Rotation2d(drivetrainSubsystem.getPigeonAngle()));
        //DriverStation.reportWarning(name + ": " + DriverStation.getAlliance().toString(), false);
        return t;
    }
    public PathPlannerTrajectory openTrajectoryFileForRed(String name, PathConstraints constraints){
        return openTrajectoryFileForAlliance(name, DriverStation.Alliance.Red);
    }
    public PathPlannerTrajectory openTrajectoryFileForBlue(String name, PathConstraints constraints){
        return openTrajectoryFileForAlliance(name, DriverStation.Alliance.Blue);
    }
}
