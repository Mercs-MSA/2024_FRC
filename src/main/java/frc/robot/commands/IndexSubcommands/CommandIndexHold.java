package frc.robot.commands.IndexSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CommandIndexHold extends Command {
  private final Intake m_intake;
  
  public CommandIndexHold(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.stopIndexMotor();
    m_intake.disableIntakeUpperSensorInterrupt();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    IntakeConstants.currentIndexState = IntakeConstants.indexState.HOLD;
  }

  @Override
  public boolean isFinished() {
    return m_intake.getIndexMotorSpeed() == 0;
  }
}

