package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVision;

public class ResetHeadingOnTag extends Command{
    private DrivetrainSubsystem mDrivetrainSubsystem;
    private PhotonVision mShooterCamera;
    private double mDefaultValue;
    public ResetHeadingOnTag(DrivetrainSubsystem dt, PhotonVision camera, double defaultValue){
        mDrivetrainSubsystem = dt;
        mShooterCamera = camera;
        mDefaultValue = (defaultValue+360) % 360.0;
        addRequirements(dt, camera);
    }

    @Override
    public void execute(){
        double heading = mShooterCamera.getRobotHeading();
        mDrivetrainSubsystem.setPigeonAngle(Math.abs(heading - mDefaultValue) < 10 ? heading : mDefaultValue);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(Math.toDegrees(mDrivetrainSubsystem.getPigeonAngle()) - mDefaultValue) < 10;
    }
}
