package frc.robot.commands.IntakeSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class CommandIntakeReverse extends Command {
  private final Intake m_intake;
  
  public CommandIntakeReverse(Intake i) {
    m_intake = i;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.reverseIntakeMotor();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(m_intake.getIntakeMotorSpeed() - IntakeConstants.kIntakeMotorSpeed) <= IntakeConstants.kIntakeMotorDCTolerance;
  }
}

