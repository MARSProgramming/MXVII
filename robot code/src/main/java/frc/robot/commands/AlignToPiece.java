package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.opencv.photo.Photo;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;

public class AlignToPiece extends Command{
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private Limelight mLimelight;
    private ProfiledPIDController snapPID;
    private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
    private PhotonVision mPhotonVision;

    public AlignToPiece(DrivetrainSubsystem drivetrainSubsystem, Limelight ll, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, PhotonVision pv) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        mLimelight = ll;
        mPhotonVision = pv;
        snapPID = drivetrainSubsystem.getAlignPieceController();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        if(DriverStation.getAlliance().isPresent() && !DriverStation.getAlliance().get().equals(alliance)){
            alliance = DriverStation.getAlliance().get();
        }

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double pieceYaw = mPhotonVision.getPieceYaw();
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        pieceYaw != 0 ? snapPID.calculate(pieceYaw/180*Math.PI, 0) : m_rotationSupplier.getAsDouble(),
                        Rotation2d.fromRadians(m_drivetrainSubsystem.getPigeonAngle()
                        + (alliance.equals(DriverStation.Alliance.Red) ? Math.PI : 0))
                )
        );
    }
}
