package frc.robot.commands.IndexSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.index.Index;

public class CommandIndexStart extends Command {
  private final Index m_index;
  
  public CommandIndexStart(Index m_index) {
    this.m_index = m_index;
    addRequirements(m_index);
  }

  @Override
  public void initialize() {
    m_index.startIndexMotor();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    IntakeConstants.currentIndexState = IntakeConstants.indexState.START;
  }

  @Override
  public boolean isFinished() {
    return true;
    //return Math.abs(m_index.getIndexMotorSpeed() + IntakeConstants.kIndexMotorSpeed) <= IntakeConstants.kIndexMotorDCTolerance;
  }
}

