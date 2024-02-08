package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class CommandIndexStart extends Command {
  private final Intake m_intake;
  
  public CommandIndexStart(Intake i) {
      m_intake = i;
      addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.startIndexMotor();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_intake.stopIndexMotor();
  }

  @Override
  public boolean isFinished() {
    return m_intake.detectNote() == false;
  }
}
