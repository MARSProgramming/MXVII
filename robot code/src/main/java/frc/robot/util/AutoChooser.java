package frc.robot.util;

import org.photonvision.PhotonVersion;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.BA0;
import frc.robot.auto.BA01;
import frc.robot.auto.BA012;
import frc.robot.auto.BA0123;
import frc.robot.auto.BA0123NP;
import frc.robot.auto.BB0;
import frc.robot.auto.BB0123NP;
import frc.robot.auto.BB02;
import frc.robot.auto.BC0;
import frc.robot.auto.BC03;
import frc.robot.auto.BD0;
import frc.robot.auto.BD08;
import frc.robot.auto.BD087;
import frc.robot.auto.BDL;
import frc.robot.auto.DoNothing;
import frc.robot.auto.RA0;
import frc.robot.auto.RA01;
import frc.robot.auto.RA012;
import frc.robot.auto.RA0123;
import frc.robot.auto.RA0123NP;
import frc.robot.auto.RA014;
import frc.robot.auto.RA0145;
import frc.robot.auto.RB0;
import frc.robot.auto.RB0123NP;
import frc.robot.auto.RB02;
import frc.robot.auto.RC0;
import frc.robot.auto.RC03;
import frc.robot.auto.RD0;
import frc.robot.auto.RD08;
import frc.robot.auto.RD087;
import frc.robot.auto.RDL;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVisionCamera;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ThePivot;

public class AutoChooser {
    private ShuffleboardTab preMatch;
    private SendableChooser<Command> autoChooser = new SendableChooser<>();
    private DrivetrainSubsystem drivetrainSubsystem;

    public AutoChooser(DrivetrainSubsystem mDrivetrainSubsystem, IntakeWheels intakeWheels, IntakePivot intakePivot, ShooterFlywheel shooterFlywheel, ThePivot thePivot, Limelight ll, PhotonVisionCamera shooterCamera){
        preMatch = Shuffleboard.getTab("Match");
        drivetrainSubsystem = mDrivetrainSubsystem;

        //auto plays
        autoChooser.setDefaultOption("Do Nothing", new DoNothing());
        autoChooser.addOption("RA0", new RA0(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RA01", new RA01(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RA012", new RA012(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RA0123", new RA0123(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RA0123NP", new RA0123NP(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RB0123NP", new RB0123NP(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RA014", new RA014(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll, shooterCamera));
        autoChooser.addOption("RA0145", new RA0145(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll, shooterCamera));
        autoChooser.addOption("RB0", new RB0(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RB02", new RB02(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RC0", new RC0(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RC03", new RC03(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RD0", new RD0(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RDL", new RDL(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RD08", new RD08(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("RD087", new RD087(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));

        autoChooser.addOption("BA0", new BA0(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BA01", new BA01(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BA012", new BA012(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BA0123", new BA0123(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BA0123NP", new BA0123NP(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BB0123NP", new BB0123NP(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        //TODO:autoChooser.addOption("BA014", new BA014(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        // autoChooser.addOption("BA0145", new BA0145(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BB0", new BB0(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BB02", new BB02(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BC0", new BC0(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BC03", new BC03(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BD0", new BD0(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BDL", new BDL(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BD08", new BD08(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));
        autoChooser.addOption("BD087", new BD087(mDrivetrainSubsystem, intakeWheels, intakePivot, shooterFlywheel, thePivot, ll));

        preMatch.add("Auto Play", autoChooser).withSize(2, 1).withPosition(4, 5);
    }

    public Command getSelected(){
        return autoChooser.getSelected();
    }

    public static PathPlannerTrajectory openTrajectoryFile(String name, DrivetrainSubsystem drivetrainSubsystem, double rotation){
        PathPlannerTrajectory t = PathPlannerPath.fromPathFile(name).getTrajectory(drivetrainSubsystem.getChassisSpeeds(), Rotation2d.fromRadians(rotation));
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
