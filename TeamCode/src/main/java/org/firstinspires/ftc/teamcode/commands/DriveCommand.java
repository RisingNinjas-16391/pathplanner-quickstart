package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
    private DrivetrainSubsystem m_drivetrain;
    private Supplier<ChassisSpeeds> m_moveSupply;
    private BooleanSupplier m_fieldRelative;

    public DriveCommand(DrivetrainSubsystem drive, Supplier<ChassisSpeeds> speedsSupplier, BooleanSupplier fieldRelativeSupplier) {
        m_drivetrain = drive;
        m_moveSupply = speedsSupplier;
        m_fieldRelative = fieldRelativeSupplier;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        ChassisSpeeds move = m_moveSupply.get();

        if (m_fieldRelative.getAsBoolean()) {
            m_drivetrain.driveFieldRelative(move);
        } else {
            m_drivetrain.driveRobotRelative(move);
        }
    }
}
