package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class ResetPose extends Command{
    private DrivetrainSubsystem mDrivetrainSubsystem;
    private Limelight mLimelight;
    private Pose2d mStartPos;
    public ResetPose(DrivetrainSubsystem dt, Limelight ll, Pose2d startPos){
        mDrivetrainSubsystem = dt;
        mLimelight = ll;
        mStartPos = startPos;

        addRequirements(dt, ll);
    }

    @Override
    public void execute(){
        if(!mLimelight.hasAprilTagTarget()){
            mDrivetrainSubsystem.setPose(mStartPos, mStartPos.getRotation());
        }
    }

    @Override
    public boolean isFinished(){
        return mDrivetrainSubsystem.getPose().getTranslation().getDistance(mStartPos.getTranslation()) < 0.5;
    }
}
