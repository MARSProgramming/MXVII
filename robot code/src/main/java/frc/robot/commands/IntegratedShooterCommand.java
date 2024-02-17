package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private boolean movePivot = false;
    private boolean shoot = false;
    private ChassisSpeeds driveSpeeds = new ChassisSpeeds();

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;

    //TODO: change to constants
    private Translation2d goalPos = new Translation2d(16.58, 5.548);
    private double[] results;
    private Timer mTimer = new Timer();
    private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
    public IntegratedShooterCommand(IntakeWheels intakeWheels, ShooterFlywheel shooterFlywheel, ThePivot thePivot, DrivetrainSubsystem dt, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        mIntakeWheels = intakeWheels;
        mShooterFlywheel = shooterFlywheel;
        mThePivot = thePivot;
        mDrivetrainSubsystem = dt;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;

        addRequirements(intakeWheels, shooterFlywheel, thePivot, dt);
    }
    public IntegratedShooterCommand(IntakeWheels intakeWheels, ShooterFlywheel shooterFlywheel, ThePivot thePivot, DrivetrainSubsystem dt) {
        mIntakeWheels = intakeWheels;
        mShooterFlywheel = shooterFlywheel;
        mThePivot = thePivot;
        mDrivetrainSubsystem = dt;
        this.m_translationXSupplier = null;
        this.m_translationYSupplier = null;

        addRequirements(intakeWheels, shooterFlywheel, thePivot, dt);
    }
    @Override
    public void initialize(){
        mDrivetrainSubsystem.getSnapController().reset(mDrivetrainSubsystem.getPigeonAngle());
        mThePivot.resetProfiledPIDController();
        movePivot = false;
        results = calculateShootingParameters(mDrivetrainSubsystem.getPose(), goalPos, mDrivetrainSubsystem.getChassisSpeeds());
    }
    @Override
    public void execute(){
        //use to shoot while moving
        //results = calculateShootingParameters(mDrivetrainSubsystem.getPose(), goalPos, mDrivetrainSubsystem.getChassisSpeeds());

        if(DriverStation.getAlliance().isPresent() && !DriverStation.getAlliance().get().equals(alliance)){
            alliance = DriverStation.getAlliance().get();
        }

        double flywheelSpeed = results[0];
        double swerveAngle = results[1];
        double pivotAngle = results[2];
        mShooterFlywheel.setVelocity(flywheelSpeed);
        if(mShooterFlywheel.atSpeed(() -> flywheelSpeed)){
            movePivot = true;
        }
        if(movePivot){
            mThePivot.setPosition(pivotAngle);
        }
        shoot = mDrivetrainSubsystem.getSnapController().getPositionError() < 5 && mShooterFlywheel.atSpeed(() -> flywheelSpeed) && mThePivot.atSetpoint() && mThePivot.belowVelocityThreshold();
        SmartDashboard.putBoolean("Flywheel At Goal", mShooterFlywheel.atSpeed(() -> flywheelSpeed));
        SmartDashboard.putBoolean("Pivot At Goal", mThePivot.atSetpoint());
        SmartDashboard.putBoolean("Drive Angle At Goal", mDrivetrainSubsystem.getSnapController().getPositionError() < 5);
        SmartDashboard.putNumber("Angle to Target", swerveAngle);
        if(shoot){
            mIntakeWheels.setDutyCycle(-0.9);
            mTimer.reset();
            mTimer.start();
        }
        double angularVelocity = mDrivetrainSubsystem.getSnapController().calculate(mDrivetrainSubsystem.getPigeonAngle(), swerveAngle)
        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        
        if(m_translationXSupplier != null && m_translationYSupplier != null){
            driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble(), 0.0, Rotation2d.fromRadians(mDrivetrainSubsystem.getPigeonAngle()
                        + (alliance.equals(DriverStation.Alliance.Red) ? Math.PI : 0)));
        }

        driveSpeeds.omegaRadiansPerSecond = Math.abs(angularVelocity) > 0.4 ? angularVelocity : 0;
        mDrivetrainSubsystem.drive(driveSpeeds);

        double dist = goalPos.getDistance(mDrivetrainSubsystem.getPose().getTranslation());
        SmartDashboard.putNumber("Distance to Goal", dist);
    }
    @Override
    public void end(boolean interrupted){
        mShooterFlywheel.setDutyCycle(0);
        mIntakeWheels.setDutyCycle(0);
        mThePivot.setDutyCycle(0);
    }
    @Override
    public boolean isFinished(){
        return !mIntakeWheels.hasPiece() && mTimer.get() > 0.2;
    }
    public static double[] calculateShootingParameters(Pose2d pos, Translation2d goalPos, ChassisSpeeds vel){
        double dist = goalPos.getDistance(pos.getTranslation());

        InterpolatingDoubleTreeMap distToTime = new InterpolatingDoubleTreeMap();
        distToTime.put(0.0, 0.0);
        distToTime.put(1.0, 0.15);
        distToTime.put(2.0, 0.3);
        distToTime.put(6.0, 0.9);
        double time = distToTime.get(dist);

        Translation2d newGoal = goalPos.minus(new Translation2d(vel.vxMetersPerSecond * time, vel.vyMetersPerSecond * time));

        dist = newGoal.getDistance(pos.getTranslation());
        InterpolatingDoubleTreeMap distToRPM = new InterpolatingDoubleTreeMap();
        distToRPM.put(1.0, 3500.0);
        distToRPM.put(2.0, 3500.0);
        distToRPM.put(3.0, 3750.0);

        InterpolatingDoubleTreeMap distToPivotAngle = new InterpolatingDoubleTreeMap();
        distToPivotAngle.put(1.5, 0.0);
        distToPivotAngle.put(2.0, 0.06);
        distToPivotAngle.put(2.43, 0.066);
        distToPivotAngle.put(2.64, 0.075);
        distToPivotAngle.put(2.9, 0.08);
        distToPivotAngle.put(3.32, 0.086);
        distToPivotAngle.put(3.7, 0.09);
        distToPivotAngle.put(4.82, 0.96);
        
        return new double[]{distToRPM.get(dist), Math.atan2(newGoal.getY()-pos.getY(), newGoal.getX()-pos.getX()), distToPivotAngle.get(dist)};
    }
}
