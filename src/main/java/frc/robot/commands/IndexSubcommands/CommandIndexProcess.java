package frc.robot.commands.IndexSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CommandIndexProcess extends Command {
  private final Intake m_intake;
  private double targetPos;

  public CommandIndexProcess(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    targetPos = m_intake.getIndexMotorPosition() - IntakeConstants.kIndexProcessRotations;
    m_intake.indexMotorToPosition(targetPos);
    IntakeConstants.currentIndexState = IntakeConstants.indexState.PROCESS;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(targetPos - m_intake.getIndexMotorPosition()) <= IntakeConstants.kIndexMotorTolerance;
  }
}

