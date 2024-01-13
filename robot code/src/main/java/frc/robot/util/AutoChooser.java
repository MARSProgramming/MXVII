package frc.robot.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.plays.DoNothing;
import frc.robot.auto.plays.Blue.BLUE_BOT_2HIGH;
import frc.robot.auto.plays.Blue.BLUE_BOT_2HIGHDOCK;
import frc.robot.auto.plays.Blue.BLUE_BOT_3PIECE;
import frc.robot.auto.plays.Blue.BLUE_BOT_LEAVE;
import frc.robot.auto.plays.Blue.BLUE_BOT_SCORELEAVE;
import frc.robot.auto.plays.Blue.BLUE_MID_LEAVE;
import frc.robot.auto.plays.Blue.BLUE_MID_LEAVEDOCK;
import frc.robot.auto.plays.Blue.BLUE_MID_SCORELEAVE;
import frc.robot.auto.plays.Blue.BLUE_MID_SCORELEAVEDOCK;
import frc.robot.auto.plays.Blue.BLUE_TOP_2HIGH;
import frc.robot.auto.plays.Blue.BLUE_TOP_2HIGHDOCK;
import frc.robot.auto.plays.Blue.BLUE_TOP_3PIECE;
import frc.robot.auto.plays.Blue.BLUE_TOP_LEAVE;
import frc.robot.auto.plays.Blue.BLUE_TOP_SCORELEAVE;
import frc.robot.auto.plays.Red.RED_BOT_2HIGH;
import frc.robot.auto.plays.Red.RED_BOT_2HIGHDOCK;
import frc.robot.auto.plays.Red.RED_BOT_3PIECE;
import frc.robot.auto.plays.Red.RED_BOT_LEAVE;
import frc.robot.auto.plays.Red.RED_BOT_SCORELEAVE;
import frc.robot.auto.plays.Red.RED_MID_LEAVE;
import frc.robot.auto.plays.Red.RED_MID_LEAVEDOCK;
import frc.robot.auto.plays.Red.RED_MID_SCORELEAVE;
import frc.robot.auto.plays.Red.RED_MID_SCORELEAVEDOCK;
import frc.robot.auto.plays.Red.RED_TOP_2HIGH;
import frc.robot.auto.plays.Red.RED_TOP_2HIGHDOCK;
import frc.robot.auto.plays.Red.RED_TOP_3PIECE;
import frc.robot.auto.plays.Red.RED_TOP_LEAVE;
import frc.robot.auto.plays.Red.RED_TOP_SCORELEAVE;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Manipulator;

public class AutoChooser {
    private ShuffleboardTab preMatch;
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutoChooser(DrivetrainSubsystem mDrivetrainSubsystem, Manipulator mManipulator){
        preMatch = Shuffleboard.getTab("Match");
        //autoChooser = new SendableChooser<>();
        

        //auto plays
        autoChooser.setDefaultOption("Do Nothing", new DoNothing());
        autoChooser.addOption("1BLUE: Leave Community", new BLUE_TOP_LEAVE(mDrivetrainSubsystem));
        autoChooser.addOption("1BLUE: Leave Score", new BLUE_TOP_SCORELEAVE(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1BLUE: 2 Piees High", new BLUE_TOP_2HIGH(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1BLUE: 2 Pieces High Dock", new BLUE_TOP_2HIGHDOCK(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1BLUE: 3 Piece", new BLUE_TOP_3PIECE(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("2BLUE: Leave Community", new BLUE_MID_LEAVE(mDrivetrainSubsystem));
        autoChooser.addOption("2BLUE: Leave Dock", new BLUE_MID_LEAVEDOCK(mDrivetrainSubsystem));
        autoChooser.addOption("2BLUE: Leave Score", new BLUE_MID_SCORELEAVE(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("2BLUE: Balance Score", new BLUE_MID_SCORELEAVEDOCK(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("3BLUE: Leave Community", new BLUE_BOT_LEAVE(mDrivetrainSubsystem));
        autoChooser.addOption("3BLUE: Leave Score", new BLUE_BOT_SCORELEAVE(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("3BLUE: 2 Pieces", new BLUE_BOT_2HIGH(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("3BLUE: 2 Pieces Dock", new BLUE_BOT_2HIGHDOCK(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("3BLUE: 3 Piece", new BLUE_BOT_3PIECE(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1RED: Leave Community", new RED_TOP_LEAVE(mDrivetrainSubsystem));
        autoChooser.addOption("1RED: Leave Score", new RED_TOP_SCORELEAVE(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1RED: 2 Piees High", new RED_TOP_2HIGH(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1RED: 2 Pieces High Dock", new RED_TOP_2HIGHDOCK(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("1RED: 3 Piece", new RED_TOP_3PIECE(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("2RED: Leave Community", new RED_MID_LEAVE(mDrivetrainSubsystem));
        autoChooser.addOption("2RED: Leave Score", new RED_MID_SCORELEAVE(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("2RED: Leave Dock", new RED_MID_LEAVEDOCK(mDrivetrainSubsystem));
        autoChooser.addOption("2RED: Balance Score", new RED_MID_SCORELEAVEDOCK(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("3RED: Leave Community", new RED_BOT_LEAVE(mDrivetrainSubsystem));
        autoChooser.addOption("3RED: Leave Score", new RED_BOT_SCORELEAVE(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("3RED: 2 Pieces", new RED_BOT_2HIGH(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("3RED: 2 Pieces Dock", new RED_BOT_2HIGHDOCK(mDrivetrainSubsystem, mManipulator));
        autoChooser.addOption("3RED: 3 Piece", new RED_BOT_3PIECE(mDrivetrainSubsystem, mManipulator));

        preMatch.add("Auto Play", autoChooser).withSize(2, 1).withPosition(4, 5);
    }

    public Command getSelected(){
        return autoChooser.getSelected();
    }

    public static PathPlannerTrajectory openTrajectoryFile(String name, PathConstraints constraints){
        PathPlannerTrajectory t = PathPlanner.loadPath(name, constraints);
        return t;
    }
    public static PathPlannerTrajectory openTrajectoryFileForAlliance(String name, PathConstraints constraints){
        PathPlannerTrajectory t = PathPlanner.loadPath(name, constraints);
        DriverStation.reportWarning(name + ": " + DriverStation.getAlliance().toString(), false);
        return PathPlannerTrajectory.transformTrajectoryForAlliance(t, DriverStation.getAlliance());
    }
    public static PathPlannerTrajectory openTrajectoryFileForRed(String name, PathConstraints constraints){
        PathPlannerTrajectory t = PathPlanner.loadPath(name, constraints);
        return PathPlannerTrajectory.transformTrajectoryForAlliance(t, DriverStation.Alliance.Red);
    }
    public static PathPlannerTrajectory openTrajectoryFileForBlue(String name, PathConstraints constraints){
        PathPlannerTrajectory t = PathPlanner.loadPath(name, constraints);
        return PathPlannerTrajectory.transformTrajectoryForAlliance(t, DriverStation.Alliance.Blue);
    }
}
