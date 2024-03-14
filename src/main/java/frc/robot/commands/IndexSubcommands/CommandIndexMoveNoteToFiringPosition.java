package frc.robot.commands.IndexSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.index.Index;

public class CommandIndexMoveNoteToFiringPosition extends Command {
  private final Index m_index;
  private double targetPos;

  public CommandIndexMoveNoteToFiringPosition(Index i) {
    m_index = i;
    addRequirements(m_index);
  }

  @Override
  public void initialize() {
    targetPos = m_index.getIndexMotorPosition() - IntakeConstants.kIndexProcessRotations;
    m_index.indexMotorToPosition(targetPos);
    IntakeConstants.currentIndexState = IntakeConstants.indexState.PROCESS;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("this index command is over", true);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(targetPos - m_index.getIndexMotorPosition()) <= IntakeConstants.kIndexMotorTolerance;
  }
}

