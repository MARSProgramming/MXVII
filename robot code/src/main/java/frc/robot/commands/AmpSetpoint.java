package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.ThePivot;

public class AmpSetpoint extends Command{
    IntakePivot mIntakePivot;
    IntakeWheels mIntakeWheels;
    ThePivot mThePivot;
    public AmpSetpoint(IntakePivot intakePivot, IntakeWheels intakeWheels, ThePivot thePivot){
        mIntakePivot = intakePivot;
        mThePivot = thePivot;
        addRequirements(thePivot, intakePivot);
    }
    @Override
    public void execute(){
        if(mThePivot.getPosition() > 0.15){
            mIntakePivot.setPosition(DynamicConstants.Intake.pivotAmpPosition);
        }
        mThePivot.setPosition(DynamicConstants.ThePivot.ampPosition);
    }
    @Override
    public void end(boolean interrupted){
        mThePivot.resetProfiledPIDController();
        mIntakePivot.resetProfiledPIDController();
        mIntakePivot.setDutyCycle(0);
        mThePivot.setDutyCycle(0);
    }
}
