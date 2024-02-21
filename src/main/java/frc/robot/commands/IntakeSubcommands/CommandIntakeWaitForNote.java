package frc.robot.commands.IntakeSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CommandIntakeWaitForNote extends Command {
  private final Intake m_intake;
  
  public CommandIntakeWaitForNote(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.startIntakeMotor();
    IntakeConstants.currentIntakeState = IntakeConstants.intakeState.INTAKE;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_intake.lowerSensorDetectsNote() == true;
  }
}

