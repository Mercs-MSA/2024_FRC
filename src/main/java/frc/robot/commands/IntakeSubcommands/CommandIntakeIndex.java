package frc.robot.commands.IntakeSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CommandIntakeIndex extends Command {
  private final Intake m_intake;
  private boolean hasSeenNote;
  
  public CommandIntakeIndex(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
    hasSeenNote = false;
  }

  @Override
  public void initialize() {
    m_intake.startIntakeMotor();
    IntakeConstants.currentIntakeState = IntakeConstants.intakeState.INDEX;
  }

  @Override
  public void execute() {
    if (m_intake.lowerSensorDetectsNote()) {
      hasSeenNote = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return hasSeenNote && !m_intake.lowerSensorDetectsNote();
  }
}

