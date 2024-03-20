package frc.robot.subsystems;

import java.sql.Driver;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionCamera extends SubsystemBase{
    private DrivetrainSubsystem dt;
    private PhotonCamera cam = new PhotonCamera("Arducam_OV9782_USB_Camera");
    private PhotonPoseEstimator photonPoseEstimator;
    EstimatedRobotPose lastPose = null;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public PhotonVisionCamera(DrivetrainSubsystem drive){
        dt = drive;

        Transform3d robotToCam = new Transform3d(new Translation3d(0.34, 0, 0), new Rotation3d(0,25,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
    }

    public void resetPose(){
        Optional<EstimatedRobotPose> pose = getEstimatedGlobalPose(dt.getPose());
        PhotonTrackedTarget trackedTarget = cam.getLatestResult().getBestTarget();
        if(cam.getLatestResult().hasTargets()){
            if(pose.isPresent() && trackedTarget != null
            && trackedTarget.getBestCameraToTarget() != null
            && ((trackedTarget.getBestCameraToTarget().getTranslation().getX() < 2.5 && (DriverStation.isDisabled() || DriverStation.isAutonomous())) 
            || (trackedTarget.getBestCameraToTarget().getTranslation().getX() < 6 && DriverStation.isTeleop())
            )){
                dt.addVisionMeasurementTimestamp(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
                lastPose = pose.get();
            }
        }
    }

    @Override
    public void periodic(){
        resetPose();
    }
    public double getRobotHeading(){
        double heading = lastPose.estimatedPose.getRotation().toRotation2d().getDegrees();
        PhotonTrackedTarget trackedTarget = cam.getLatestResult().getBestTarget();
        if(cam.getLatestResult().hasTargets()){
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
