package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AimAtGoal extends Command{
    private Pose2d goal;
    private ChassisSpeeds driveSpeeds = new ChassisSpeeds();
    private DrivetrainSubsystem mDrivetrainSubsystem;
    public AimAtGoal(Pose2d g, DrivetrainSubsystem dt){
        mDrivetrainSubsystem = dt;
        goal = g;
    }
    @Override
    public void initialize(){
    }
    @Override
    public void execute(){
        Pair<Double, Double> results = calculateFlywheelRPM(mDrivetrainSubsystem.getPose(), goal.getTranslation());
        double flywheelSpeed = results.getFirst();
        double swerveAngle = results.getSecond();
        
        double angularVelocity = mDrivetrainSubsystem.getSnapController().calculate(mDrivetrainSubsystem.getPigeonAngle(), swerveAngle)
        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        
        driveSpeeds.omegaRadiansPerSecond = angularVelocity;
        mDrivetrainSubsystem.drive(driveSpeeds);
    }

    public static Pair<Double, Double> calculateFlywheelRPM(Pose2d pos, Translation2d goalPos){
        InterpolatingDoubleTreeMap distToRPM = new InterpolatingDoubleTreeMap();
        distToRPM.put(1.0, 100.0);
        distToRPM.put(2.0, 150.0);
        distToRPM.put(3.0, 190.0);

        double dist = goalPos.getDistance(pos.getTranslation());
        return new Pair<Double,Double>(distToRPM.get(dist), Math.atan2(goalPos.getY()-pos.getY(), goalPos.getX()-pos.getX()));
    }
    public static Pair<Double, Double> calculateFlywheelRPMMoving(Pose2d pos, Translation2d goalPos, Translation2d vel){
        InterpolatingDoubleTreeMap distToTime = new InterpolatingDoubleTreeMap();
        distToTime.put(1.0, 0.2);
        distToTime.put(2.0, 0.5);
        distToTime.put(3.0, 0.7);

        double dist = goalPos.getDistance(pos.getTranslation());
        double time = distToTime.get(dist);
        return calculateFlywheelRPM(pos, goalPos.minus(new Translation2d(vel.getX() * time, vel.getY() * time)));
    }
}
