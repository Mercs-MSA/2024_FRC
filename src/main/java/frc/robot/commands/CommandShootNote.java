package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants.ScoringMode;
import frc.robot.subsystems.SAT.SAT;

public class CommandShootNote extends Command {
  ScoringMode target;
  double shooterSpeed;
  public SAT m_SAT;

  public CommandShootNote(SAT s) {
        this(s, ScoringConstants.currentScoringMode);
  }

  public CommandShootNote(SAT s, ScoringMode t) {
    target = t;
    m_SAT = s;
    addRequirements(m_SAT);

    switch (target) {
        case PODIUM:
            shooterSpeed = Constants.SATConstants.PODIUM.shooterSpeed;
            break;
        case SUB:
            shooterSpeed = Constants.SATConstants.SUBWOOFER.shooterSpeed;
            break;
        case AMP:
            shooterSpeed = Constants.SATConstants.AMP.shooterSpeed;
            break;
        case WING:
            shooterSpeed = Constants.SATConstants.WING.shooterSpeed;
            break;
    }
  }

  @Override
  public void initialize() {
    m_SAT.shootNote(shooterSpeed);
  }
  
  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
   
  }
  @Override
  public boolean isFinished() {
    return Math.abs(m_SAT.getShooterSpeed() - shooterSpeed) <= SATConstants.kShooterSpeedTolerance;
  }

}