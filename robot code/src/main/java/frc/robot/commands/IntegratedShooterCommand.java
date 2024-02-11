package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ShooterFlywheel;

public class IntegratedShooterCommand extends Command {
    private IntakeWheels mIntakeWheels;
    private ShooterFlywheel mShooterFlywheel;
    public IntegratedShooterCommand(IntakeWheels intakeWheels, ShooterFlywheel shooterFlywheel) {
        mIntakeWheels = intakeWheels;
        mShooterFlywheel = shooterFlywheel;
    }
    @Override
    public void execute(){
        mShooterFlywheel.setVelocity(DynamicConstants.ShooterFlywheel.testVelocity);
        if(mShooterFlywheel.atSpeed(() -> DynamicConstants.ShooterFlywheel.testVelocity)){
            mIntakeWheels.runDutyCycle(-0.4);
        }
    }
    @Override
    public void end(boolean interrupted){
        mShooterFlywheel.runDutyCycle(0);
        mIntakeWheels.runDutyCycle(0);
    }
}
