package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CommandSwerveGoToHeading extends Command {
    
      private Rotation2d heading;
      private Swerve m_swerve;
  
  public CommandSwerveGoToHeading(double target, Swerve s) {
    setSubsystem("Swerve");
    heading = Rotation2d.fromDegrees(target);
    m_swerve = s;
  }

  @Override
  public void initialize() {
  }
  
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Go To Heading", 0);
  }
  
  @Override
  public void execute() {
    SmartDashboard.putNumber("Current Heading", m_swerve.getHeading().getDegrees());
    SmartDashboard.putNumber("Go To Heading", heading.getDegrees());
    m_swerve.drive(m_swerve.getPose().getTranslation(), heading.getRadians(), true, true);
  }

  @Override
  public boolean isFinished() {
    return m_swerve.getHeading() == heading;
  }
    
}
