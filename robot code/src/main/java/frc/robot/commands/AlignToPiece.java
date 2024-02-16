package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignToPiece extends Command{
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private Limelight mLimelight;
    private ProfiledPIDController snapPID;
    private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;

    public AlignToPiece(DrivetrainSubsystem drivetrainSubsystem, Limelight ll, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        mLimelight = ll;
        snapPID = drivetrainSubsystem.getAlignPieceController();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        if(DriverStation.getAlliance().isPresent() && !DriverStation.getAlliance().get().equals(alliance)){
            alliance = DriverStation.getAlliance().get();
        }

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        mLimelight.pieceLLhasTarget() ? snapPID.calculate(mLimelight.getPiecePosition()/180*Math.PI, 0) : m_rotationSupplier.getAsDouble(),
                        Rotation2d.fromRadians(m_drivetrainSubsystem.getPigeonAngle()
                        + (alliance.equals(DriverStation.Alliance.Red) ? Math.PI : 0))
                )
        );
    }
}
