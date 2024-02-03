package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class CommandIntakeStop extends Command {
  public Intake m_intake;

  public CommandIntakeStop(Intake i) {
    setSubsystem("Intake");
    m_intake = i;
  }

  @Override
  public void initialize() {
    m_intake.stopIntakeMotor();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return m_intake.getIntakeMotor() == 0.0;
  }
}

