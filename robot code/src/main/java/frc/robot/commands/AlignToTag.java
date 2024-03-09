package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.StaticConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignToTag extends Command{
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private Limelight mLimelight;
    
    public AlignToTag(DrivetrainSubsystem drivetrainSubsystem, Limelight ll) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        mLimelight = ll;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                        0,
                        mLimelight.getTrapTagX() * StaticConstants.Drive.kPTagAlignX,
                        mLimelight.getTrapTagRot() * StaticConstants.Drive.kPTagAlignRot
                )
        );
    }

    // @Override
    // public boolean isFinished(){
    //     return mLimelight.hasTrapTag() && (Math.abs(mLimelight.getTrapTagX()) < 0.04 && Math.abs(mLimelight.getTrapTagRot()) < 10);
    // }
}
