// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RioConstants;
import frc.robot.constants.StaticConstants;

public class DrivetrainSubsystem extends SubsystemBase {
    //private ShuffleboardTab Match = Shuffleboard.getTab("Match");

    public static final double MAX_VOLTAGE = 12.0;
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4I_L3.getDriveReduction() *
            SdsModuleConfigurations.MK4I_L3.getWheelDiameter() * Math.PI;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(StaticConstants.Drive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    StaticConstants.Drive.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(StaticConstants.Drive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    StaticConstants.Drive.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(StaticConstants.Drive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -StaticConstants.Drive.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-StaticConstants.Drive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    StaticConstants.Drive.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-StaticConstants.Drive.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -StaticConstants.Drive.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    private SwerveModule m_frontLeftModule;
    private SwerveModule m_frontRightModule;
    private SwerveModule m_backLeftModule;
    private SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DrivetrainSubsystem() {
        double fr = RioConstants.Drive.FRONT_RIGHT_MODULE_STEER_OFFSET;
        double fl = RioConstants.Drive.FRONT_LEFT_MODULE_STEER_OFFSET;
        double br = RioConstants.Drive.BACK_RIGHT_MODULE_STEER_OFFSET;
        double bl = RioConstants.Drive.BACK_LEFT_MODULE_STEER_OFFSET;

        initSnapController();
        createSwerveModules(fl, fr, bl, br);
        SmartDashboard.putData("Zero Swerves", zeroSwerveCommand().ignoringDisable(true));

        mPoseEstimator = new SwerveDrivePoseEstimator(m_kinematics, new Rotation2d(getPigeonAngle()),
                getSwerveModulePositions(), new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(15)));
    }

    //swerve
    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());

        mPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d(getPigeonAngle()),
                getSwerveModulePositions());
        
        SmartDashboard.putNumber("Pigeon Angle", getPigeonAngle() * 180.0/Math.PI);
        SmartDashboard.putNumber("Angular Velocity", m_chassisSpeeds.omegaRadiansPerSecond);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                new SwerveModulePosition(m_frontLeftModule.getPosition(),
                        new Rotation2d(m_frontLeftModule.getSteerAngle())),
                new SwerveModulePosition(m_frontRightModule.getPosition(),
                        new Rotation2d(m_frontRightModule.getSteerAngle())),
                new SwerveModulePosition(m_backLeftModule.getPosition(),
                        new Rotation2d(m_backLeftModule.getSteerAngle())),
                new SwerveModulePosition(m_backRightModule.getPosition(),
                        new Rotation2d(m_backRightModule.getSteerAngle()))
        };
    }

    //zero swerve command
    public Command zeroSwerveCommand(){
        return run(() -> {
            System.out.println("zeroing");
            RioConstants.writeSwerveZeros(m_frontLeftModule.getSteerAngle(),
            m_frontRightModule.getSteerAngle(),
            m_backLeftModule.getSteerAngle(),
            m_backRightModule.getSteerAngle());
        });
    }

    //heading snap controller
    private ProfiledPIDController mSnapController;
    private void initSnapController(){
        mSnapController = new ProfiledPIDController(StaticConstants.Drive.kP,
                StaticConstants.Drive.kI,
                StaticConstants.Drive.kD,
                new TrapezoidProfile.Constraints(StaticConstants.Auto.holonomicOMaxVelocity,
                        StaticConstants.Auto.holonomicOMaxAcceleration));
        mSnapController.enableContinuousInput(-Math.PI, Math.PI);
    }
    public ProfiledPIDController getSnapController() {
        return mSnapController;
    }

    //auto drive controller
    private HolonomicDriveController mHolonomicDriveController = new HolonomicDriveController(
            new PIDController(StaticConstants.Auto.holonomicXkP, StaticConstants.Auto.holonomicXkI,
                    StaticConstants.Auto.holonomicXkD),
            new PIDController(StaticConstants.Auto.holonomicYkP, StaticConstants.Auto.holonomicYkI,
                    StaticConstants.Auto.holonomicYkD),
            new ProfiledPIDController(StaticConstants.Auto.holonomicOkP, StaticConstants.Auto.holonomicOkI,
                    StaticConstants.Auto.holonomicOkD,
                    new TrapezoidProfile.Constraints(StaticConstants.Auto.holonomicOMaxVelocity,
                            StaticConstants.Auto.holonomicOMaxAcceleration)));
    
    public HolonomicDriveController getDrivePathController() {
        return mHolonomicDriveController;
    }

    //Pigeon
    private final Pigeon2 m_pigeon = new Pigeon2(StaticConstants.Drive.DRIVETRAIN_PIGEON_ID);

    public double getPigeonAngle() {
        return Math.toRadians(m_pigeon.getYaw().getValueAsDouble());
    }
    public Command zeroGyroscope(double d) {
        return runOnce(() -> {
            m_pigeon.setYaw(d);
            mSnapController.reset(Math.toRadians(d));
        });
    }
    public double getRoll() {
        return m_pigeon.getRoll().getValueAsDouble();
    }

    //PoseEstimator
    private final SwerveDrivePoseEstimator mPoseEstimator;

    public void addVisionMeasurement(Pose2d pose, double latencySeconds) {
        mPoseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - latencySeconds);
    }
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> mat) {
        mPoseEstimator.setVisionMeasurementStdDevs(mat);
    }
    public Pose2d getPose() {
        return mPoseEstimator.getEstimatedPosition();
    }
    public void setPose(Pose2d pose, Rotation2d rotation) {
        mPoseEstimator.resetPosition(rotation, getSwerveModulePositions(), pose);
    }
    public double getTranslationalSpeed() {
        return Math.sqrt(Math.pow(m_chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(m_chassisSpeeds.vyMetersPerSecond, 2));
    }
    public ChassisSpeeds getChassisSpeeds(){
        return m_chassisSpeeds;
    }
    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }
    

    public void createSwerveModules(double fl, double fr, double bl, double br) {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();
        config.setCanivoreName(StaticConstants.Drive.kDriveCANivore);

        m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                config,
                Mk4iSwerveModuleHelper.GearRatio.L3,
                15,
                StaticConstants.Drive.FRONT_LEFT_MODULE_STEER_MOTOR,
                StaticConstants.Drive.FRONT_LEFT_MODULE_STEER_ENCODER,
                fl);

        m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                config,
                Mk4iSwerveModuleHelper.GearRatio.L3,
                14,
                StaticConstants.Drive.FRONT_RIGHT_MODULE_STEER_MOTOR,
                StaticConstants.Drive.FRONT_RIGHT_MODULE_STEER_ENCODER,
                fr);
        m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                config,
                Mk4iSwerveModuleHelper.GearRatio.L3,
                16,
                StaticConstants.Drive.BACK_LEFT_MODULE_STEER_MOTOR,
                StaticConstants.Drive.BACK_LEFT_MODULE_STEER_ENCODER,
                bl);
        m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                config,
                Mk4iSwerveModuleHelper.GearRatio.L3,
                17,
                StaticConstants.Drive.BACK_RIGHT_MODULE_STEER_MOTOR,
                StaticConstants.Drive.BACK_RIGHT_MODULE_STEER_ENCODER,
                br);
    }
}

