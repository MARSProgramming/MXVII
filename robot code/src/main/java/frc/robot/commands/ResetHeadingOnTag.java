package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionCamera;

public class ResetHeadingOnTag extends Command{
    private DrivetrainSubsystem mDrivetrainSubsystem;
    private PhotonVisionCamera mShooterCamera;
    private double mDefaultValue;
    public ResetHeadingOnTag(DrivetrainSubsystem dt, PhotonVisionCamera camera, double defaultValue){
        mDrivetrainSubsystem = dt;
        mShooterCamera = camera;
        mDefaultValue = defaultValue;
        addRequirements(dt, camera);
    }

    @Override
    public void execute(){
        double heading = mShooterCamera.getRobotHeading();
        System.out.println(heading);
        mDrivetrainSubsystem.setPigeonAngle(Math.abs(heading - mDefaultValue) < 5 ? heading : mDefaultValue);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(Math.toDegrees(mDrivetrainSubsystem.getPigeonAngle()) - mDefaultValue) < 5;
    }
}
