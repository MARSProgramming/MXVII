package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private Translation2d redGoalPos = new Translation2d(16.58, 5.548);
    private Translation2d blueGoalPos = new Translation2d(0, 5.548);
    private double[] results;
    private Timer mTimer = new Timer();
    private Timer mPushTimer = new Timer();
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
        mTimer.stop();
        mTimer.reset();
        if(DriverStation.getAlliance().isPresent() && !DriverStation.getAlliance().get().equals(alliance)){
            alliance = DriverStation.getAlliance().get();
        }
        results = calculateShootingParameters(mDrivetrainSubsystem.getPose(), alliance.equals(Alliance.Red) ? redGoalPos : blueGoalPos, mDrivetrainSubsystem.getChassisSpeeds(), this.alliance);
    }
    @Override
    public void execute(){
        // if(mIntakeWheels.hasPiece()){
        //     mIntakeWheels.setDutyCycle(-0.15);
        // }
        //use to shoot while moving
        if(mDrivetrainSubsystem.getTranslationalSpeed() > 0.05){
            results = calculateShootingParameters(mDrivetrainSubsystem.getPose(), alliance.equals(Alliance.Red) ? redGoalPos : blueGoalPos, mDrivetrainSubsystem.getChassisSpeeds(), this.alliance);
            mDrivetrainSubsystem.setSnapTolerance(0.2);
        }
        else{
            mDrivetrainSubsystem.setSnapTolerance(0.08);
        }

        double flywheelSpeed = results[0];
        double swerveAngle = results[1];
        double pivotAngle = results[2];
        mShooterFlywheel.setVelocity(flywheelSpeed);
        if(mShooterFlywheel.atSpeed(() -> flywheelSpeed) && Math.abs(mDrivetrainSubsystem.getSnapController().getGoal().position - mDrivetrainSubsystem.getPigeonAngle()) < 0.3){
            movePivot = true;
        }
        if(movePivot){
            mThePivot.setPosition(pivotAngle);
        }
        shoot = mDrivetrainSubsystem.getSnapController().atGoal() && mShooterFlywheel.atSpeed(() -> flywheelSpeed) && mThePivot.atSetpoint() && mThePivot.belowVelocityThreshold();
        SmartDashboard.putBoolean("Flywheel At Goal", mShooterFlywheel.atSpeed(() -> flywheelSpeed));
        SmartDashboard.putBoolean("Pivot At Goal", mThePivot.atSetpoint());
        SmartDashboard.putBoolean("Drive Angle At Goal", mDrivetrainSubsystem.getSnapController().atGoal());
        SmartDashboard.putNumber("Snap Error", mDrivetrainSubsystem.getSnapController().getPositionError());
        if(shoot){
            mIntakeWheels.setDutyCycle(-1);
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
    }
    @Override
    public void end(boolean interrupted){
        mShooterFlywheel.setDutyCycle(0);
        mIntakeWheels.setDutyCycle(0);
        mThePivot.setDutyCycle(0);
    }
    @Override
    public boolean isFinished(){
        return !mIntakeWheels.hasPiece() && mTimer.get() > 0.4;
    }
    public static double[] calculateShootingParameters(Pose2d pos, Translation2d goalPos, ChassisSpeeds vel, Alliance alliance){
        double dist = goalPos.getDistance(pos.getTranslation());

        InterpolatingDoubleTreeMap distToTime = new InterpolatingDoubleTreeMap();
        distToTime.put(0.0, 0.0);
        distToTime.put(1.36, 0.1);
        distToTime.put(2.8, 0.296);
        distToTime.put(4.22, 0.38);
        double time = distToTime.get(dist);

        Translation2d newGoal = goalPos.minus(new Translation2d(vel.vxMetersPerSecond * time, vel.vyMetersPerSecond * time));

        dist = newGoal.getDistance(pos.getTranslation());
        InterpolatingDoubleTreeMap distToRPM = new InterpolatingDoubleTreeMap();
        distToRPM.put(1.0, 3500.0);
        distToRPM.put(2.0, 3500.0);
        distToRPM.put(3.0, 3750.0);
        distToRPM.put(3.5, 3750.0);
        distToRPM.put(4.0, 4500.0);
        distToRPM.put(4.5, 4500.0);
        distToRPM.put(5.0, 5500.0);
        distToRPM.put(7.0, 5500.0);

        InterpolatingDoubleTreeMap distToPivotAngle = new InterpolatingDoubleTreeMap();
        distToPivotAngle.put(1.5, 0.0);
        distToPivotAngle.put(2.0, 0.055);
        distToPivotAngle.put(2.43, 0.066);
        distToPivotAngle.put(2.64, 0.075);
        distToPivotAngle.put(2.9, 0.079);
        distToPivotAngle.put(3.32, 0.084);
        distToPivotAngle.put(3.7, 0.097);
        distToPivotAngle.put(4.0, 0.1);
        distToPivotAngle.put(4.3, 0.11);
        distToPivotAngle.put(4.5, 0.115);
        distToPivotAngle.put(5.0, 0.122);
        distToRPM.put(5.5, 0.125);
        
        double angleOffset = alliance.equals(Alliance.Red) ? -0.12 : 0.12;
        SmartDashboard.putNumber("Dist To Goal", dist);
        //TODO: set fudge factor as dynamic constant
        return new double[]{distToRPM.get(dist), Math.atan2(newGoal.getY()-pos.getY(), (newGoal.getX() + angleOffset - pos.getX())), Math.max(distToPivotAngle.get(dist) - 0.01, 0)};
    }
}
