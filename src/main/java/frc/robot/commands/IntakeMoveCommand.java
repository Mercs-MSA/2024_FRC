package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeMoveCommand extends Command {
  public Intake m_intake;
  public double intakeArmPositionTarget;
  
  public IntakeMoveCommand(double target) {
    intakeArmPositionTarget = target;
    setSubsystem("Intake");
  }

  @Override
  public void initialize() {
    m_intake.m_fx.setControl(m_intake.m_voltagePosition.withPosition(intakeArmPositionTarget));
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.m_fx.setControl(m_intake.m_brake);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    // TODO This line needs to be better
    return m_intake.m_fx.getPosition().getValue() == intakeArmPositionTarget;
  }
}
