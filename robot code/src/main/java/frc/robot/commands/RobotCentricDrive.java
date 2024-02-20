package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotCentricDrive extends Command {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;

    /*
     * Creates a default command to drive a swerve drive subsystem with supplied 
     * DrivetrainSubsystem and DoubleSuppliers as dynamic inputs.
     * 
     * @param  drivetrainSubsystem   the DrivetrainSubsystem to drive
     * @param  translationXSupplier  the supplied X axis translational input
     * @param  translationYSupplier  the supplied Y axis translational input
     * @param  rotationSupplier      the supplied rotational input
     */
    public RobotCentricDrive(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        0,
                        0
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
