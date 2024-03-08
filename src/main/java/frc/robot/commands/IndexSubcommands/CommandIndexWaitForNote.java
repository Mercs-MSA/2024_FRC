package frc.robot.commands.IndexSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CommandIndexWaitForNote extends Command {
  private final Intake m_intake;
  
  public CommandIndexWaitForNote(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.enableAsynchronousInterrupt();
    m_intake.startIndexMotor();
    IntakeConstants.currentIntakeState = IntakeConstants.intakeState.INDEX;
    
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_intake.disableAsynchronousInterrupt();
  }

  @Override
  public boolean isFinished() {
    return m_intake.upperSensorDetectsNote() == true;
  }
}

