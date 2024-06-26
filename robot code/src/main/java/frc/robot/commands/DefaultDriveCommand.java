package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends Command {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;

    /*
     * Creates a default command to drive a swerve drive subsystem with supplied 
     * DrivetrainSubsystem and DoubleSuppliers as dynamic inputs.
     * 
     * @param  drivetrainSubsystem   the DrivetrainSubsystem to drive
     * @param  translationXSupplier  the supplied X axis translational input
     * @param  translationYSupplier  the supplied Y axis translational input
     * @param  rotationSupplier      the supplied rotational input
     */
    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if(DriverStation.getAlliance().isPresent() && !DriverStation.getAlliance().get().equals(alliance)){
            alliance = DriverStation.getAlliance().get();
        }

        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        Rotation2d.fromRadians(m_drivetrainSubsystem.getPigeonAngle()
                        + (alliance.equals(DriverStation.Alliance.Red) ? Math.PI : 0))
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted){
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        }
    }
}
