
package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class DriveAtPath extends Command {
    private final DrivetrainSubsystem mDrivetrainSubsystem;
    private PathPlannerTrajectory mTrajectory;
    private HolonomicDriveController mController;
    private Limelight mLimelight;
    private Timer mTimer = new Timer();
    private ProfiledPIDController snapPID;
    private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;

    public DriveAtPath(DrivetrainSubsystem subsystem, PathPlannerTrajectory traj, Limelight ll) {
        mTrajectory = traj;
        mDrivetrainSubsystem = subsystem;
        mController = subsystem.getDrivePathController();
        mLimelight = ll;
        snapPID = subsystem.getSnapController();
        
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //transformedTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(mTrajectory, DriverStation.getAlliance());
        mController.setTolerance(new Pose2d(0.01, 0.01, new Rotation2d(0.05)));
        mTimer.reset();
        mTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        PathPlannerTrajectory.State state = mTrajectory.sample(mTimer.get() + 0.3);

        if(DriverStation.getAlliance().isPresent() && !DriverStation.getAlliance().get().equals(alliance)){
            alliance = DriverStation.getAlliance().get();
        }

        ChassisSpeeds speeds = mController.calculate(mDrivetrainSubsystem.getPose(), state.getTargetHolonomicPose(), state.velocityMps, state.targetHolonomicRotation);
        /*if(mLimelight.pieceLLhasTarget()){
            speeds.omegaRadiansPerSecond = snapPID.calculate(mLimelight.getPiecePosition()/180*Math.PI, 0);  
        }*/
        mDrivetrainSubsystem.drive(speeds);
        SmartDashboard.putNumber("desiredX", state.positionMeters.getX());
        SmartDashboard.putNumber("autoXError", mDrivetrainSubsystem.getPose().getX() - state.positionMeters.getX());
        SmartDashboard.putNumber("desiredY", state.positionMeters.getY());
        SmartDashboard.putNumber("autoYError", mDrivetrainSubsystem.getPose().getY() - state.positionMeters.getY());
        SmartDashboard.putNumber("desiredrot", state.targetHolonomicRotation.getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mDrivetrainSubsystem.drive(new ChassisSpeeds());
        mTimer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mDrivetrainSubsystem.getPose().getTranslation().getDistance(mTrajectory.getEndState().positionMeters) < 0.05;
    }
}
