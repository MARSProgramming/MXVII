package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Dynamic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DynamicConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;

public class IntakeAmp extends Command{
    IntakePivot mIntakePivot;
    IntakeWheels mIntakeWheels;
    private Timer mTimer = new Timer();
    public IntakeAmp(IntakePivot intakePivot, IntakeWheels intakeWheels){
        mIntakePivot = intakePivot;
        mIntakeWheels = intakeWheels;
        addRequirements(intakeWheels, intakePivot);
    }
    @Override
    public void initialize(){
        mTimer.reset();
        mTimer.stop();
    }
    @Override
    public void execute(){
        mIntakePivot.setVoltage(DynamicConstants.Intake.intakeAmpPivotVoltage);
        if(mIntakePivot.getPosition() > DynamicConstants.Intake.pivotIntakeAmpPosition){
            mIntakeWheels.setVoltage(DynamicConstants.Intake.intakeAmpOuttakeVoltage);
            mTimer.start();
        }
    }
    @Override
    public void end(boolean interrupted){
        mIntakePivot.setVoltage(0);
        mIntakeWheels.setVoltage(0);
    }
    @Override
    public boolean isFinished(){
        return mTimer.get() > 0.2;
    }
}
