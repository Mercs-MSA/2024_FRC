package frc.robot.commands.IndexSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CommandIndexProcess extends Command {
  private final Intake m_intake;
  
  public CommandIndexProcess(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.indexMotorToPosition(IntakeConstants.kIndexProcessRotations);
    IntakeConstants.currentIndexState = IntakeConstants.indexState.PROCESS;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(IntakeConstants.kIndexProcessRotations - m_intake.getIndexMotorPosition()) <= IntakeConstants.kIndexMotorTolerance;
  }
}

