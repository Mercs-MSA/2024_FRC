package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CommandOverrideIntakeStop extends Command {
  private final Intake m_intake;

  public CommandOverrideIntakeStop(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.stopIntakeMotor();
  }
  
  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    IntakeConstants.currentIntakeState = IntakeConstants.intakeState.OVERRIDE_MOTOR_OFF;
  }

  @Override
  public boolean isFinished() {
    return m_intake.getIntakeMotorSpeed() == 0.0;
  }
}
