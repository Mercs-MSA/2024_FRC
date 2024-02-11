package frc.robot.commands.IndexSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CommandIndexFire extends Command {
  private final Intake m_intake;
  private boolean hasSeenNote;
  
  public CommandIndexFire(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
    hasSeenNote = false;
  }

  @Override
  public void initialize() {
    m_intake.startIndexMotor();
    IntakeConstants.currentIndexState = IntakeConstants.indexState.FIRE;
  }

  @Override
  public void execute() {
    if (m_intake.upperSensorDetectsNote()) {
      hasSeenNote = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return hasSeenNote && !m_intake.upperSensorDetectsNote();
  }
}

