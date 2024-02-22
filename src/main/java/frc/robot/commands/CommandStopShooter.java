package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SAT.SAT;

public class CommandStopShooter extends Command {
  public SAT m_SAT;

  public CommandStopShooter(SAT s) {
    m_SAT = s;
    addRequirements(m_SAT);
  }

  @Override
  public void initialize() {
    m_SAT.stopShooter();
  }
  
  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
   
  }
  @Override
  public boolean isFinished() {
    return m_SAT.getShooterSpeed() == 0.0;
  }

}