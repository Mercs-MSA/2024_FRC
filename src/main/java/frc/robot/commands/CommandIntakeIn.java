package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class CommandIntakeIn extends Command {
  public Intake m_intake;
  
  public CommandIntakeIn(Intake i) {
    setSubsystem("Intake");
    m_intake = i;
  }

  @Override
  public void initialize() {
    m_intake.feedToShooter();
  }

  @Override
  public void end(boolean interrupted) {
    // m_intake.stopIntakeMotor();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return m_intake.isNotePresent == true;
  }
}

