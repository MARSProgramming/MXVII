package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.StaticConstants;

public class Limelight extends SubsystemBase{
    double[] botpose;
    private DrivetrainSubsystem dt;
    private Timer pieceLLTimer = new Timer(); 
    public Limelight(DrivetrainSubsystem drive){
        dt = drive;
    }

    public boolean hasAprilTagTarget(){
        double dist = NetworkTableInstance.getDefault().getTable("limelight-trap").getEntry("targetpose_cameraspace").getDoubleArray(new double[7])[2];

        return NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("tv").getDouble(0) == 1.0 || (NetworkTableInstance.getDefault().getTable("limelight-trap").getEntry("tv").getDouble(0) == 1.0 && dist < 2.5);
    }

    public void resetPose(){
        botpose = NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("botpose").getDoubleArray(new double[7]);
        
        double x = botpose[0] + StaticConstants.Field.fieldLength/2.0;
        double y = botpose[1] + StaticConstants.Field.fieldWidth/2.0;

        boolean hasShooterTarget = NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("tv").getDouble(0) == 1.0;
        double dist = NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("targetpose_cameraspace").getDoubleArray(new double[7])[2];
        if(dist < 3 && hasShooterTarget && (DriverStation.isTeleopEnabled() || DriverStation.isDisabled())){
            dt.addVisionMeasurement(new Pose2d(x, y, Rotation2d.fromRadians(dt.getPigeonAngle())), botpose[6]/1000);
        }
        if(hasShooterTarget && DriverStation.isAutonomousEnabled() && dist < 3 && dt.getPose().getTranslation().getDistance(new Translation2d(x, y)) < 0.5){
            dt.addVisionMeasurement(new Pose2d(x, y, Rotation2d.fromRadians(dt.getPigeonAngle())), botpose[6]/1000);
        }

        botpose = NetworkTableInstance.getDefault().getTable("limelight-trap").getEntry("botpose").getDoubleArray(new double[7]);
        
        x = botpose[0] + StaticConstants.Field.fieldLength/2.0;
        y = botpose[1] + StaticConstants.Field.fieldWidth/2.0;

        boolean hasPieceTarget = NetworkTableInstance.getDefault().getTable("limelight-trap").getEntry("tv").getDouble(0) == 1.0;
        dist = NetworkTableInstance.getDefault().getTable("limelight-trap").getEntry("targetpose_cameraspace").getDoubleArray(new double[7])[2];
        if(dist < 2.5 && hasPieceTarget && (DriverStation.isDisabled())){
            dt.addVisionMeasurement(new Pose2d(x, y, Rotation2d.fromRadians(dt.getPigeonAngle())), botpose[6]/1000);
        }
        // if(hasPieceTarget && DriverStation.isAutonomousEnabled() && dist < 2 && dt.getPose().getTranslation().getDistance(new Translation2d(x, y)) < 0.5){
        //     dt.addVisionMeasurement(new Pose2d(x, y, Rotation2d.fromRadians(dt.getPigeonAngle())), botpose[6]/1000);
        // }
    }

    @Override
    public void periodic(){
        resetPose();
        if(NetworkTableInstance.getDefault().getTable("limelight-piece").getEntry("tv").getDouble(0) != 1.0){
            pieceLLTimer.reset();
            pieceLLTimer.start();
        }
    }

    public boolean pieceLLhasTarget(){
        return pieceLLTimer.get() > 0.25;
    }
    public boolean hasTrapTag(){
        //TODO: make it not dumb
        if(NetworkTableInstance.getDefault().getTable("limelight-trap").getEntry("tv").getDouble(0) == 1.0){
            return true;
        }
        return false;
    }
    public double getTrapTagX(){
        if(NetworkTableInstance.getDefault().getTable("limelight-trap").getEntry("tv").getDouble(0) != 1.0){
            return 0.0;
        }
        return NetworkTableInstance.getDefault().getTable("limelight-trap").getEntry("targetpose_cameraspace").getDoubleArray(new double[7])[0];
    }
    public double getTrapTagY(){
        if(NetworkTableInstance.getDefault().getTable("limelight-trap").getEntry("tv").getDouble(0) != 1.0){
            return 0.0;
        }
        return NetworkTableInstance.getDefault().getTable("limelight-trap").getEntry("targetpose_cameraspace").getDoubleArray(new double[7])[1];
    }
    public double getTrapTagRot(){
        if(NetworkTableInstance.getDefault().getTable("limelight-trap").getEntry("tv").getDouble(0) != 1.0){
            return 0.0;
        }
        return NetworkTableInstance.getDefault().getTable("limelight-trap").getEntry("targetpose_cameraspace").getDoubleArray(new double[7])[4];
    }
    public DoubleSupplier getHeadingFromShooterLL(double defaultvalue){
        botpose = NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("botpose").getDoubleArray(new double[7]);
        return () -> (NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("tv").getDouble(0) != 1.0 && Math.abs(botpose[5] - defaultvalue) < 10 ? defaultvalue : botpose[5]);
    }
    public double getPiecePosition(){
        if(pieceLLhasTarget()){
            return NetworkTableInstance.getDefault().getTable("limelight-piece").getEntry("tx").getDouble(0);
        }
        return 0;
    }
}
