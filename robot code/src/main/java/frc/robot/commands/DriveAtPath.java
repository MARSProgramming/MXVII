
package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;

public class DriveAtPath extends Command {
    private final DrivetrainSubsystem mDrivetrainSubsystem;
    private PathPlannerTrajectory mTrajectory;
    private HolonomicDriveController mController;
    private Limelight mLimelight;
    private Timer mTimer = new Timer();
    private ProfiledPIDController piecePID;
    private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
    private boolean alignToPiece = false;
    private PhotonVision mPhotonVision;
    private BooleanSupplier mHasPiece;

    private StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("Desired Pose", Pose2d.struct).publish();
    private Field2d m_field = new Field2d();

    public DriveAtPath(DrivetrainSubsystem subsystem, PathPlannerTrajectory traj, Limelight ll, boolean alignToPiece, PhotonVision pv) {
        this(subsystem, traj, ll, alignToPiece, pv, () -> false);
    }

    public DriveAtPath(DrivetrainSubsystem subsystem, PathPlannerTrajectory traj, Limelight ll, boolean alignToPiece, PhotonVision pv, BooleanSupplier hasPiece) {
        mTrajectory = traj;
        mDrivetrainSubsystem = subsystem;
        mController = subsystem.getDrivePathController();
        mLimelight = ll;
        mPhotonVision = pv;
        piecePID = subsystem.getAlignPieceController();
        this.alignToPiece = alignToPiece;
        SmartDashboard.putData("Desired Pose", m_field);
        mHasPiece = hasPiece;


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
        if(alignToPiece && mPhotonVision.getPieceYaw() != 0.0 && mTimer.get() > mTrajectory.getTotalTimeSeconds()/3 && !mHasPiece.getAsBoolean()){
            speeds.omegaRadiansPerSecond = piecePID.calculate(mPhotonVision.getPieceYaw()/180*Math.PI, 0);  
        }
        mDrivetrainSubsystem.drive(speeds);
        SmartDashboard.putNumber("desiredX", state.positionMeters.getX());
        SmartDashboard.putNumber("desiredY", state.positionMeters.getY());
        publisher.set(state.getTargetHolonomicPose());
        SmartDashboard.putNumber("desiredrot", state.targetHolonomicRotation.getRadians());
        m_field.setRobotPose(state.getTargetHolonomicPose());
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
        return mDrivetrainSubsystem.getPose().getTranslation().getDistance(mTrajectory.getEndState().positionMeters) < 0.05 && mTimer.get() > mTrajectory.getTotalTimeSeconds() - 0.5;
    }
}
