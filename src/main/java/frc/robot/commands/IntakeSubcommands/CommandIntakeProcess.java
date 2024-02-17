package frc.robot.commands.IntakeSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CommandIntakeProcess extends Command {
  private final Intake m_intake;
  private double targetPos;
  
  public CommandIntakeProcess(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    targetPos = m_intake.getIntakeMotorPosition() - IntakeConstants.kIntakeProcessRotations;
    m_intake.intakeMotorToPosition(targetPos);
    IntakeConstants.currentIntakeState = IntakeConstants.intakeState.PROCESS;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(targetPos - m_intake.getIntakeMotorPosition()) <= IntakeConstants.kIntakeMotorTolerance;
  }
}

