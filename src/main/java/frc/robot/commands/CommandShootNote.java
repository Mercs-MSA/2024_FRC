package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SATConstants;
import frc.robot.subsystems.SAT.SAT;

public class CommandShootNote extends Command {
  public SAT m_SAT;

  public CommandShootNote(SAT s) {
    m_SAT = s;
    addRequirements(m_SAT);
  }

  @Override
  public void initialize() {
    m_SAT.shootNote();
  }
  
  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
   
  }
  @Override
  public boolean isFinished() {
    return Math.abs(m_SAT.getShooterSpeed() - SATConstants.SHOOTER_SPEED) <= SATConstants.kShooterSpeedTolerance;
  }

}