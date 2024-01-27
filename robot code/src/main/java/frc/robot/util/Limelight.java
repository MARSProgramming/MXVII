package frc.robot.util;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Limelight {
    private static Limelight mInstance;
    public static Limelight getInstance(){
        if(mInstance == null) mInstance = new Limelight();
        return mInstance;
    }
    private Limelight(){}

    private DrivetrainSubsystem dt;
    private Timer pieceLLTimer = new Timer(); 
    public void setDrivetrain(DrivetrainSubsystem drive){
        dt = drive;
    }

    public boolean pieceLLhasTarget(){
        return pieceLLTimer.get() > 0.25;
    }
    public double getPiecePosition(){
        if(pieceLLhasTarget()){
            return NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tx").getDouble(0);
        }
        return 0;
    }

    public void periodic(){
        if(NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tv").getDouble(0) != 1.0){
            pieceLLTimer.start();
        }
    }
}
