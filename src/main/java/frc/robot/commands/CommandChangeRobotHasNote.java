package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

public class CommandChangeRobotHasNote extends Command {
    private final boolean value;
  
  public CommandChangeRobotHasNote(boolean v) {
    value = v;
  }

  @Override
  public void initialize() {
    IntakeConstants.kRobotHasNote = value;
  }

  @Override
  public boolean isFinished(){
    return true;
  }
}

