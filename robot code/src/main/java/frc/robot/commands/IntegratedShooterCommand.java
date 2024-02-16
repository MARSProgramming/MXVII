package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ThePivot;

public class IntegratedShooterCommand extends Command {
    private IntakeWheels mIntakeWheels;
    private ShooterFlywheel mShooterFlywheel;
    private ThePivot mThePivot;
    private DrivetrainSubsystem mDrivetrainSubsystem;
    private ChassisSpeeds driveSpeeds = new ChassisSpeeds();
    private Translation2d goalPos = new Translation2d(16.58, 5.548);
    private double[] results;
    private Timer mTimer = new Timer();
    public IntegratedShooterCommand(IntakeWheels intakeWheels, ShooterFlywheel shooterFlywheel, ThePivot thePivot, DrivetrainSubsystem dt) {
        mIntakeWheels = intakeWheels;
        mShooterFlywheel = shooterFlywheel;
        mThePivot = thePivot;
        mDrivetrainSubsystem = dt;
    }
    @Override
    public void initialize(){
        mDrivetrainSubsystem.getSnapController().reset(mDrivetrainSubsystem.getPigeonAngle());
        mThePivot.resetProfiledPIDController();
        results = calculateFlywheelRPM(mDrivetrainSubsystem.getPose(), goalPos);
    }
    @Override
    public void execute(){

        // double flywheelSpeed = results[0];
        // double swerveAngle = results[1];
        // double pivotAngle = results[2];
        // mShooterFlywheel.setVelocity(DynamicConstants.ShooterFlywheel.testVelocity);
        // if(mShooterFlywheel.atSpeed(() -> DynamicConstants.ShooterFlywheel.testVelocity)){
        //    // System.out.println(DynamicConstants.ThePivot.testPosition);
        //     mThePivot.setPosition(DynamicConstants.ThePivot.testPosition);
        // }
        // if(mShooterFlywheel.atSpeed(() -> DynamicConstants.ShooterFlywheel.testVelocity) && mThePivot.atSetpoint()){
        //     mIntakeWheels.setDutyCycle(-0.8);
        // }
        // double angularVelocity = mDrivetrainSubsystem.getSnapController().calculate(mDrivetrainSubsystem.getPigeonAngle(), swerveAngle)
        // * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        
        // driveSpeeds.omegaRadiansPerSecond = angularVelocity;
        // mDrivetrainSubsystem.drive(driveSpeeds);

        // double dist = goalPos.getDistance(mDrivetrainSubsystem.getPose().getTranslation());
        // System.out.println(dist);
        double flywheelSpeed = results[0];
        double swerveAngle = results[1];
        double pivotAngle = results[2];
        mShooterFlywheel.setVelocity(flywheelSpeed);
        if(mShooterFlywheel.atSpeed(() -> flywheelSpeed)){
            mThePivot.setPosition(pivotAngle);
        }
        if(mShooterFlywheel.atSpeed(() -> flywheelSpeed) && mThePivot.belowVelocityThreshold() && mThePivot.atSetpoint()){
            mIntakeWheels.setDutyCycle(-0.9);
            mTimer.reset();
            mTimer.start();
        }
        double angularVelocity = mDrivetrainSubsystem.getSnapController().calculate(mDrivetrainSubsystem.getPigeonAngle(), swerveAngle)
        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        
        driveSpeeds.omegaRadiansPerSecond = angularVelocity;
        mDrivetrainSubsystem.drive(driveSpeeds);

        double dist = goalPos.getDistance(mDrivetrainSubsystem.getPose().getTranslation());
        System.out.println(dist);
    }
    @Override
    public void end(boolean interrupted){
        mShooterFlywheel.setDutyCycle(0);
        mIntakeWheels.setDutyCycle(0);
        mThePivot.setDutyCycle(0);
    }
    @Override
    public boolean isFinished(){
        return !mIntakeWheels.hasPiece() && mTimer.get() > 0.3;
    }
    public static double[] calculateFlywheelRPM(Pose2d pos, Translation2d goalPos){
        InterpolatingDoubleTreeMap distToRPM = new InterpolatingDoubleTreeMap();
        distToRPM.put(1.0, 3500.0);
        distToRPM.put(2.0, 3500.0);
        distToRPM.put(3.0, 3750.0);

        InterpolatingDoubleTreeMap distToPivotAngle = new InterpolatingDoubleTreeMap();
        distToPivotAngle.put(1.0, 0.0);
        distToPivotAngle.put(2.0, 0.041);
        distToPivotAngle.put(2.43, 0.051);
        distToPivotAngle.put(2.64, 0.063);
        distToPivotAngle.put(2.9, 0.071);
        distToPivotAngle.put(3.32, 0.08);
        distToPivotAngle.put(4.82, 0.083);

        double dist = goalPos.getDistance(pos.getTranslation());
        
        return new double[]{distToRPM.get(dist), Math.atan2(goalPos.getY()-pos.getY(), goalPos.getX()-pos.getX()), distToPivotAngle.get(dist)};
    }
}
