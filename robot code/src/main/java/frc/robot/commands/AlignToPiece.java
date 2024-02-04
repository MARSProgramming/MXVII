package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignToPiece extends Command{
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private Limelight mLimelight;
    private ProfiledPIDController snapPID;

    public AlignToPiece(DrivetrainSubsystem drivetrainSubsystem, Limelight ll, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        mLimelight = ll;
        snapPID = drivetrainSubsystem.getSnapController();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        -m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        mLimelight.pieceLLhasTarget() ? snapPID.calculate(mLimelight.getPiecePosition()/180*Math.PI, 0) : 0,
                        Rotation2d.fromRadians(m_drivetrainSubsystem.getPigeonAngle())
                )
        );
    }
}
