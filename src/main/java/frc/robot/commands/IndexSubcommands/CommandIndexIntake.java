package frc.robot.commands.IndexSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CommandIndexIntake extends Command {
  private final Intake m_intake;
  
  public CommandIndexIntake(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.startIndexMotor();
    IntakeConstants.currentIndexState = IntakeConstants.indexState.INTAKE;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_intake.upperSensorDetectsNote() == true;
  }
}

