package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.StaticConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterFlywheel;

public class DefaultShooterCommand extends Command{
    private DrivetrainSubsystem mDrivetrainSubsystem;
    private ShooterFlywheel mShooterFlywheel;
    private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
    public DefaultShooterCommand(ShooterFlywheel flywheel, DrivetrainSubsystem dt){
        mDrivetrainSubsystem = dt;
        mShooterFlywheel = flywheel;

        addRequirements(flywheel);
    }

    @Override
    public void execute(){
        if(DriverStation.getAlliance().isPresent() && !DriverStation.getAlliance().get().equals(alliance)){
            alliance = DriverStation.getAlliance().get();
        }
        double x = mDrivetrainSubsystem.getPose().getX();
        if((alliance.equals(DriverStation.Alliance.Blue) && x < StaticConstants.Field.fieldLength/2.0) || (alliance.equals(DriverStation.Alliance.Red) && x > StaticConstants.Field.fieldLength/2.0) && x > 0.1){
            mShooterFlywheel.setVelocity(DynamicConstants.ShooterFlywheel.idleVelocity);
        }
        else{
            mShooterFlywheel.setDutyCycle(0);
        }
    }
}
