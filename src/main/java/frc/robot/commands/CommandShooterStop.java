package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class CommandShooterStop extends Command {
  public Shooter m_shooter;

  public CommandShooterStop(Shooter s) {
    m_shooter = s;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.stopShooter(); //neutral out
  }
  
  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
  }
  @Override
  public boolean isFinished() {
    //return isWithinTol(0.0, m_shooter.getShooterSpeed(), 25); 
    return true;
  }


  public boolean isWithinTol(double targetPose, double currentPose, double tolerance) {
    return (Math.abs(targetPose - currentPose) <= tolerance);
  }
}