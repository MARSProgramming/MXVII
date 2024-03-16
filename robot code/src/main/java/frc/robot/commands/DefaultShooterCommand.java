package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.StaticConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ShooterFlywheel;

public class DefaultShooterCommand extends Command{
    private DrivetrainSubsystem mDrivetrainSubsystem;
    private ShooterFlywheel mShooterFlywheel;
    private IntakeWheels mIntakeWheels;
    private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
    private boolean climbing = false;
    private Climber mClimber;
    public DefaultShooterCommand(ShooterFlywheel flywheel, DrivetrainSubsystem dt, Climber climber, IntakeWheels intakeWheels){
        mDrivetrainSubsystem = dt;
        mShooterFlywheel = flywheel;
        mClimber = climber;
        mIntakeWheels = intakeWheels;

        addRequirements(flywheel);
    }

    @Override
    public void execute(){
        if(DriverStation.getAlliance().isPresent() && !DriverStation.getAlliance().get().equals(alliance)){
            alliance = DriverStation.getAlliance().get();
        }
        double x = mDrivetrainSubsystem.getPose().getX();
        if(((alliance.equals(DriverStation.Alliance.Blue) && x < StaticConstants.Field.fieldLength/2.0) || (alliance.equals(DriverStation.Alliance.Red) && x > StaticConstants.Field.fieldLength/2.0)) && x > 0.1 && !climbing && mIntakeWheels.hasPiece()){
            mShooterFlywheel.setVelocity(DynamicConstants.ShooterFlywheel.idleVelocity);
        }
        else{
            mShooterFlywheel.setDutyCycle(0);
        }
        if(mClimber.getLeftPosition() > 1 || mClimber.getRightPosition() > 1){
            climbing = true;
        }
    }
}
