package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SAT;

public class CommandShooterStop extends Command {
  public SAT m_SAT;

  public CommandShooterStop(SAT s) {
    m_SAT = s;
    addRequirements(m_SAT);
  }

  @Override
  public void initialize() {
    m_SAT.stopShooter(); //neutral out
  }
  
  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    m_SAT.stopShooter();
  }
  @Override
  public boolean isFinished() {
    //return isWithinTol(0.0, m_SAT.getShooterSpeed(), 25); 
    return true;
  }


  public boolean isWithinTol(double targetPose, double currentPose, double tolerance) {
    return (Math.abs(targetPose - currentPose) <= tolerance);
  }
}