package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class CommandDriveStraight extends Command {
    // public Swerve s_Swerve;
    private Rotation2d initialYaw;

    public CommandDriveStraight() {
        // s_Swerve = s;
        // addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        initialYaw = Swerve.poseEstimator.getEstimatedPosition().getRotation();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("CommandDriveStraight", -(initialYaw.getDegrees() - Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees()));
        // s_Swerve.drive(new Translation2d(0, 0), -(initialYaw.getDegrees() - s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees()), false, false);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("shooter command is done", true);
    }

    @Override
    public boolean isFinished() {
        return false;   
    }
}