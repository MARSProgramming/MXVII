package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    double[] botpose;
    private DrivetrainSubsystem dt;
    private Timer pieceLLTimer = new Timer(); 
    public Limelight(DrivetrainSubsystem drive){
        dt = drive;
    }

    public boolean hasTarget(){
        return false;
    }

    public void resetPose(){
        botpose = NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        double x = botpose[0];
        double y = botpose[1];

        double dist = NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("targetpose_cameraspace").getDoubleArray(new double[7])[2];
        if(dist < 3 && NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("tv").getDouble(0) == 1.0){
            dt.addVisionMeasurement(new Pose2d(x, y, Rotation2d.fromRadians(dt.getPigeonAngle())), botpose[6]/1000);
        }
    }

    @Override
    public void periodic(){
        resetPose();
        if(NetworkTableInstance.getDefault().getTable("limelight-piece").getEntry("tv").getDouble(0) != 1.0){
            pieceLLTimer.start();
        }
    }

    public boolean pieceLLhasTarget(){
        return pieceLLTimer.get() > 0.25;
    }
    public double getPiecePosition(){
        if(pieceLLhasTarget()){
            return NetworkTableInstance.getDefault().getTable("limelight-piece").getEntry("tx").getDouble(0);
        }
        return 0;
    }
}
