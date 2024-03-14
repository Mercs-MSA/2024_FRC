package frc.robot.commands.IndexSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.index.Index;

public class CommandIndexWaitForNote extends Command {
  private final Index m_index;
  
  public CommandIndexWaitForNote(Index i) {
    m_index = i;
    addRequirements(m_index);
  }

  @Override
  public void initialize() {
    m_index.enableAsynchronousInterrupt();
    m_index.startIndexMotor();
    IntakeConstants.currentIntakeState = IntakeConstants.intakeState.INDEX;
    
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_index.disableAsynchronousInterrupt();
  }

  @Override
  public boolean isFinished() {
    return m_index.upperSensorDetectsNote() == true;
  }
}

