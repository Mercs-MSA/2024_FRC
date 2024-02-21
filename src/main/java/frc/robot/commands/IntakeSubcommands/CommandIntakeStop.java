package frc.robot.commands.IntakeSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants.IntakeConstants;

public class CommandIntakeStop extends Command {
  private final Intake m_intake;
  
  public CommandIntakeStop(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
  }
  
  @Override
  public void initialize() {
    m_intake.stopIntakeMotor();
  }
  
  @Override
  public void execute() {}
  
  @Override
  public void end(boolean interrupted) {
    IntakeConstants.currentIntakeState = IntakeConstants.intakeState.IDLE;
  }

  @Override
  public boolean isFinished() {
    return m_intake.getIntakeMotorSpeed() == 0;
  }
}

