package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CommandOverrideIndexStop extends Command {
  private final Intake m_intake;

  public CommandOverrideIndexStop(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.stopIndexMotor();
  }
  
  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    IntakeConstants.currentIndexState = IntakeConstants.indexState.OVERRIDE_MOTOR_OFF;
  }

  @Override
  public boolean isFinished() {
    return m_intake.getIndexMotorSpeed() == 0.0;
  }
}

