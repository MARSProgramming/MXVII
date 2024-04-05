// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToPiece;
import frc.robot.commands.AlignToTag;
import frc.robot.commands.AmpSetpoint;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultShooterCommand;
import frc.robot.commands.GoToZero;
import frc.robot.commands.InitializeClimbSetpoint;
import frc.robot.commands.InitializeTrapSetpoint;
import frc.robot.commands.IntakeAmp;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntegratedShooterCommand;
import frc.robot.commands.RobotCentricDrive;
import frc.robot.commands.Test.DisableLimitsCommand;
import frc.robot.commands.Test.ResetPositions;
import frc.robot.commands.Test.SwitchNeutralModeCommand;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.SubsystemIO;
import frc.robot.subsystems.ThePivot;
import frc.robot.util.AutoChooser;
import frc.robot.util.Rumble;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();
    private final IntakePivot mIntakePivot = new IntakePivot();
    private final IntakeWheels mIntakeWheels = new IntakeWheels();
    private final ShooterFlywheel mShooterFlywheel = new ShooterFlywheel();
    private final ThePivot mThePivot = new ThePivot();
    private final Climber mClimber = new Climber();
    private final Limelight mLimelight = new Limelight(mDrivetrainSubsystem);
    private final PhotonVision mPhotonVisionCamera = new PhotonVision(mDrivetrainSubsystem);
    private final SubsystemIO subsystemIO = new SubsystemIO(mDrivetrainSubsystem, mIntakeWheels, mIntakePivot, mShooterFlywheel, mThePivot, mClimber);
    private final AutoChooser autoChooser = new AutoChooser(mDrivetrainSubsystem, mIntakeWheels, mIntakePivot, mShooterFlywheel, mThePivot, mLimelight, mPhotonVisionCamera);
    private final LED m_Led = new LED();

    public LED getLED(){
      return m_Led;
    }

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController mPilot =
        new CommandXboxController(0);
    private final CommandXboxController mCopilot = 
        new CommandXboxController(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        Rumble.getInstance().setControllers(0, 1);
        mDrivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            mDrivetrainSubsystem,
            () -> -modifyAxis(mPilot.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

        mShooterFlywheel.setDefaultCommand(new DefaultShooterCommand(mShooterFlywheel, mDrivetrainSubsystem, mClimber, mIntakeWheels));
        // Configure the trigger bindings
        configureBindings();
        configureLEDTriggers();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */

    private void configureLEDTriggers(){
      new Trigger(() -> mIntakeWheels.hasPiece()).whileTrue(m_Led.setOrangeFlashing().withTimeout(1).andThen(m_Led.setOrange()));
      mPilot.rightTrigger().whileTrue(m_Led.setYellowFlashing());
      m_Led.setDefaultCommand(m_Led.setRed());
    }
    private void configureBindings() {
        mPilot.y().whileTrue(mDrivetrainSubsystem.zeroGyroscope(0));
        // mPilot.leftTrigger().whileTrue(new AlignToPiece(mDrivetrainSubsystem, mLimelight,
        //     () -> -modifyAxis(mPilot.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        //     () -> -modifyAxis(mPilot.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        //     () -> -modifyAxis(mPilot.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, mPhotonVisionCamera));
        //mPilot.rightTrigger().whileTrue(mShooterFlywheel.runVelocity(() -> 6000.0));
        //mPilot.leftTrigger().whileTrue(mIntakePivot.setPositionCommand(() -> DynamicConstants.Intake.pivotIntakePosition, false));
        
        mPilot.leftTrigger().whileTrue(new IntakeCommand(mIntakePivot, mIntakeWheels, mThePivot));
        mPilot.leftTrigger().toggleOnFalse(mIntakePivot.zeroIntake());
        mPilot.start().whileTrue(mIntakeWheels.runVoltage(-4)); 
        mPilot.start().whileTrue(mShooterFlywheel.runVoltage(4));
        mPilot.leftBumper().whileTrue(mShooterFlywheel.runVoltage(-7).alongWith(mIntakeWheels.runVoltage(6)));
        mPilot.povLeft().whileTrue(mIntakePivot.runVoltage(-1));
        mPilot.povRight().whileTrue(mIntakePivot.runVoltage(1));
        mPilot.povUp().whileTrue(mThePivot.runVoltage(-1));
        mPilot.povDown().whileTrue(mThePivot.runVoltage(1));
        mPilot.b().whileTrue(new RobotCentricDrive(mDrivetrainSubsystem, () -> modifyAxis(mPilot.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND));
        mPilot.a().whileTrue(mShooterFlywheel.runVelocity(() -> 3300).alongWith(new WaitCommand(1).andThen(mIntakeWheels.outtake())));
        mPilot.x().whileTrue(new AlignToTag(mDrivetrainSubsystem, mLimelight).andThen(m_Led.setPurple()));
        mPilot.rightBumper().whileTrue(new IntakeAmp(mIntakePivot, mIntakeWheels).andThen(mIntakePivot.zeroIntake()));
        mPilot.rightBumper().whileTrue(mShooterFlywheel.runVoltage(0));

       
        mPilot.rightTrigger().whileTrue(
            new IntegratedShooterCommand(
                    mIntakeWheels, mShooterFlywheel, mThePivot, mDrivetrainSubsystem, 
                    () -> -modifyAxis(mPilot.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
                    () -> -modifyAxis(mPilot.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
                )
            .andThen(mThePivot.setPositionCommand(() -> DynamicConstants.ThePivot.zeroPosition, false))
        );

        mPilot.back().whileTrue(mIntakeWheels.runVoltage(6));


        // copilot
        mCopilot.leftTrigger().whileTrue(new AmpSetpoint(mIntakePivot, mIntakeWheels, mThePivot));
        mCopilot.leftBumper().whileTrue(new GoToZero(mIntakePivot, mThePivot));
        mCopilot.rightBumper().whileTrue(mIntakeWheels.runVoltage(DynamicConstants.Intake.trapOuttakeVoltage));
        mCopilot.rightTrigger().whileTrue(mShooterFlywheel.runVelocity(() -> 4000).alongWith(new WaitCommand(1).andThen(mIntakeWheels.outtake())));
        mCopilot.b().whileTrue(mClimber.climbToLimit().andThen(new InitializeTrapSetpoint(mIntakePivot, mThePivot).alongWith(new WaitCommand(1).andThen(mIntakeWheels.runVoltage(DynamicConstants.Intake.trapOuttakeVoltage)))));
        mCopilot.x().whileTrue(mClimber.setPositionCommand(() -> DynamicConstants.Climber.uprightPosition).alongWith(mIntakeWheels.runVoltage(3)));
        mCopilot.a().toggleOnTrue(new InitializeClimbSetpoint(mIntakePivot, mThePivot));
        mCopilot.y().toggleOnTrue(new InitializeTrapSetpoint(mIntakePivot, mThePivot));

        mCopilot.start().whileTrue(mClimber.runOneSideVoltageOverrideLimit(-2, true));
        mCopilot.back().whileTrue(mClimber.runOneSideVoltageOverrideLimit(-2, false));
        mCopilot.povUp().whileTrue(mClimber.runVoltage(1));
        mCopilot.povDown().whileTrue(mClimber.runVoltage(-1));
        mCopilot.povRight().whileTrue(mIntakePivot.runVoltage(-0.5));
        mCopilot.povLeft().whileTrue(mIntakePivot.runVoltage(1));
    }

    public void configureTestBindings() {
        mShooterFlywheel.removeDefaultCommand();

        CommandXboxController mTestPilot =
        new CommandXboxController(2);

        // PIVOT CONTROLS
        mTestPilot.povUp().whileTrue(mThePivot.runVoltage(-1));
        mTestPilot.povDown().whileTrue(mThePivot.runVoltage(1));
        mTestPilot.povRight().whileTrue(mIntakePivot.runVoltage(1));
        mTestPilot.povLeft().whileTrue(mIntakePivot.runVoltage(-1));

        // FLYWHEEL AND INTAKE WHEEL CONTROLS
        mTestPilot.rightBumper().whileTrue(mIntakeWheels.runVoltage(1));
        mTestPilot.leftBumper().whileTrue(mIntakeWheels.runVoltage(-1));
        mTestPilot.rightTrigger().whileTrue(mShooterFlywheel.runVoltage(1));
        mTestPilot.leftTrigger().whileTrue(mShooterFlywheel.runVoltage(-1));

        // CLIMBER CONTROLS
        new Trigger(() -> mTestPilot.getLeftY() > 0.7).whileTrue(mClimber.runOneSideVoltage(-1, true));
        new Trigger(() -> mTestPilot.getLeftY() < -0.7).whileTrue(mClimber.runOneSideVoltage(1, true));
        new Trigger(() -> mTestPilot.getRightY() > 0.7).whileTrue(mClimber.runOneSideVoltage(-1, false));
        new Trigger(() -> mTestPilot.getRightY() < -0.7).whileTrue(mClimber.runOneSideVoltage(1, false));

        // DISABLE SOFTWARE LIMITS
        mTestPilot.x().onTrue(new DisableLimitsCommand(mIntakePivot, mThePivot, mClimber));

        // SWITCH NEUTRAL MODE
        mTestPilot.b().onTrue(new SwitchNeutralModeCommand(mIntakePivot, mThePivot));

        // RESET POSITIONS
        mTestPilot.start().onTrue(new ResetPositions(mIntakePivot, mThePivot, mClimber));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
    
          }
        } else {
          return 0.0;
        }
      }
    
      private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        //"Stage" mode
        //value = Math.round(value * 5.0)/5.0;
    
        return value;
      }
}
