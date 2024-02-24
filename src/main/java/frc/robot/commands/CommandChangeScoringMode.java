package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringConstants;

public class CommandChangeScoringMode extends Command {
    private final ScoringConstants.ScoringMode target;
  
  public CommandChangeScoringMode(ScoringConstants.ScoringMode t) {
    target = t;
  }

  @Override
  public void initialize() {
    ScoringConstants.currentScoringMode = target;
  }

  @Override
  public boolean isFinished(){
    return true;
  }
}

