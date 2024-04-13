package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase{
    private DrivetrainSubsystem dt;
    private PhotonCamera shooterCam = new PhotonCamera("shooter");
    private PhotonCamera pieceCam = new PhotonCamera("piece");
    private PhotonPoseEstimator photonPoseEstimator;
    EstimatedRobotPose lastPose = null;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public PhotonVision(DrivetrainSubsystem drive){
        dt = drive;

        Transform3d robotToCam = new Transform3d(new Translation3d(0.34, 0, 0), new Rotation3d(0,25,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, shooterCam, robotToCam);
    }

    public void resetPose(){
        Optional<EstimatedRobotPose> pose = getEstimatedGlobalPose(dt.getPose());
        PhotonPipelineResult latestResult = shooterCam.getLatestResult();
        if(latestResult.hasTargets()){
            PhotonTrackedTarget trackedTarget = latestResult.getBestTarget();
            
            if(pose.isPresent() && trackedTarget != null
            && trackedTarget.getBestCameraToTarget() != null
            && ((trackedTarget.getBestCameraToTarget().getTranslation().getX() < 3.5 && (DriverStation.isDisabled() || DriverStation.isAutonomous())) 
            || (trackedTarget.getBestCameraToTarget().getTranslation().getX() < 6 && DriverStation.isTeleop())
            )){
                double stdev = 0.01 * trackedTarget.getBestCameraToTarget().getTranslation().getX();
                dt.setVisionMeasurementStdDevs(VecBuilder.fill(stdev, stdev, 0.5));
                dt.addVisionMeasurementTimestamp(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
                lastPose = pose.get();
            }
        }
    }

    public double getPieceYaw(){
        PhotonPipelineResult latestResult = pieceCam.getLatestResult();
        return latestResult.hasTargets() && latestResult.getBestTarget() != null ? latestResult.getBestTarget().getYaw() : 0;
    }

    @Override
    public void periodic(){
        resetPose();
        //System.out.println(pieceCam.getLatestResult().targets.toString());
        //System.out.println(pieceCam.getLatestResult().hasTargets() ? pieceCam.getLatestResult().getBestTarget().getYaw() : 0);
    }
    public double getRobotHeading(){
        double heading = 0;
        if(lastPose != null){
            heading = lastPose.estimatedPose.getRotation().toRotation2d().getDegrees();
        }
        PhotonPipelineResult plresult = shooterCam.getLatestResult();
        if(plresult.getBestTarget() != null && plresult.hasTargets()){
            PhotonTrackedTarget trackedTarget = plresult.getBestTarget();
            if(trackedTarget != null && trackedTarget.getBestCameraToTarget() != null && aprilTagFieldLayout.getTagPose(trackedTarget.getFiducialId()).isPresent()){
                heading = (aprilTagFieldLayout.getTagPose(trackedTarget.getFiducialId()).get().getRotation().toRotation2d().getDegrees() - (trackedTarget.getBestCameraToTarget().getRotation().toRotation2d().getDegrees()));
            }
        }
        return heading;
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
