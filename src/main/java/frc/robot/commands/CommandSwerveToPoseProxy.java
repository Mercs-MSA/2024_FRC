package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class CommandSwerveToPoseProxy extends InstantCommand {
    private Swerve s_Swerve;

    private final DoubleSupplier tx;
    private final DoubleSupplier ty;
    private final DoubleSupplier yaw;

    public CommandSwerveToPoseProxy(Swerve s_Swerve, DoubleSupplier tx, DoubleSupplier ty, DoubleSupplier yaw) {
        this.s_Swerve = s_Swerve;

        this.tx = tx;
        this.ty = ty;
        this.yaw = yaw;

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        s_Swerve.driveToPose(
            new Pose2d(
                tx.getAsDouble(),
                ty.getAsDouble(),
                Rotation2d.fromDegrees(yaw.getAsDouble())
            )
        ).schedule();
    }
}