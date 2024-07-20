package frc.robot.commands.IndexSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class CommandIndexReverse extends Command {
  private final Intake m_intake;
  
  public CommandIndexReverse(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.reverseIndexMotor();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    IntakeConstants.currentIndexState = IntakeConstants.indexState.START;
  }

  @Override
  public boolean isFinished() {
    return true;
    //return Math.abs(m_intake.getIndexMotorSpeed() + IntakeConstants.kIndexMotorSpeed) <= IntakeConstants.kIndexMotorDCTolerance;
  }
}

