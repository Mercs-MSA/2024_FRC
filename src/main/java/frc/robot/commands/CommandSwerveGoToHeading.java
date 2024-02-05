package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;

public class CommandSwerveGoToHeading extends Command {
  private final Rotation2d heading;
  private final Swerve m_swerve;
  
  public CommandSwerveGoToHeading(double target, Swerve s) {
    heading = Rotation2d.fromDegrees(target);
    m_swerve = s;
    addRequirements(m_swerve);
  }

  @Override
  public void initialize() {

  }
  
  @Override
  public void execute() {
    m_swerve.faceHeading(heading);
  }
  
  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return m_swerve.getHeading() == heading;
  }
}
