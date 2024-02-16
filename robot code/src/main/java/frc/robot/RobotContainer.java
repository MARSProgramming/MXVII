// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToPiece;
import frc.robot.commands.AmpSetpoint;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.GoToZero;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Test.DisableLimitsCommand;
import frc.robot.commands.Test.ResetPositions;
import frc.robot.commands.Test.SwitchNeutralModeCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Limelight;
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
    private final SubsystemIO subsystemIO = new SubsystemIO(mDrivetrainSubsystem, mIntakeWheels, mIntakePivot, mShooterFlywheel, mThePivot, mClimber);
    private final AutoChooser autoChooser = new AutoChooser(mDrivetrainSubsystem, mIntakeWheels, mIntakePivot, mShooterFlywheel, mThePivot);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController mPilot =
        new CommandXboxController(0);
    //private final CommandXboxController mCopilot = 
        //new CommandXboxController(1);

    private final CommandXboxController mTestPilot =
        new CommandXboxController(2);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        Rumble.getInstance().setControllers(0, 1);

        mDrivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            mDrivetrainSubsystem,
            () -> -modifyAxis(mPilot.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

        // Configure the trigger bindings
        configureBindings();
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
    private void configureBindings() {
        mPilot.y().whileTrue(mDrivetrainSubsystem.zeroGyroscope(0));
        /*mPilot.leftTrigger().whileTrue(new AlignToPiece(mDrivetrainSubsystem, mLimelight,
            () -> -modifyAxis(mPilot.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mPilot.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));*/
        //mPilot.rightTrigger().whileTrue(mShooterFlywheel.runVelocity(() -> 6000.0));
        mPilot.leftTrigger().whileTrue(mIntakePivot.setPositionCommand(() -> 0.55, true));
        mPilot.rightTrigger().whileTrue(mIntakePivot.setPositionCommand(() -> 0.4, false));
        mPilot.rightBumper().whileTrue(mIntakeWheels.runVoltage(-4));
        mPilot.povLeft().whileTrue(mIntakePivot.runVoltage(-1.5));
        mPilot.povRight().whileTrue(mIntakePivot.runVoltage(1.5));
        mPilot.povUp().whileTrue(mThePivot.runVoltage(-1));
        mPilot.povDown().whileTrue(mThePivot.runVoltage(1));
        mPilot.b().whileTrue(new AmpSetpoint(mIntakePivot, mIntakeWheels, mThePivot));
        mPilot.a().whileTrue(new GoToZero(mIntakePivot, mThePivot));
        mPilot.x().whileTrue(mShooterFlywheel.runVelocity(() -> 4000.0));
        mPilot.leftBumper().whileTrue(mShooterFlywheel.runVelocity(() -> 3000.0));
        mPilot.start().whileTrue(mIntakeWheels.runVoltage(10.5));


        // copilot
        //mCopilot.rightBumper().whileTrue(mClimber.runVoltage());
        //mCopilot.leftBumper().whileTrue(mClimber.runVoltageNegative());



    }

    public void configureTestBindings() {
        // PIVOT CONTROLS
        mTestPilot.povUp().whileTrue(mIntakePivot.runVoltage(1));
        mTestPilot.povDown().whileTrue(mIntakePivot.runVoltage(-1));
        mTestPilot.povRight().whileTrue(mThePivot.runVoltage(1));
        mTestPilot.povLeft().whileTrue(mThePivot.runVoltage(-1));

        // FLYWHEEL AND INTAKE WHEEL CONTROLS
        mTestPilot.rightBumper().whileTrue(mIntakeWheels.runVoltage(1));
        mTestPilot.leftBumper().whileTrue(mIntakeWheels.runVoltage(-1));
        mTestPilot.rightTrigger().whileTrue(mShooterFlywheel.runVoltage(1));
        mTestPilot.leftBumper().whileTrue(mShooterFlywheel.runVoltage(-1));

        // CLIMBER CONTROLS
        mTestPilot.y().whileTrue(mClimber.runVoltage(1));
        mTestPilot.a().whileTrue(mClimber.runVoltage(-1));

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
