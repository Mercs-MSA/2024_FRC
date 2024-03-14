package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class CommandShooterReverse extends Command {
    public Shooter m_shooter;

    public CommandShooterReverse(Shooter s) {
        m_shooter = s;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.shootNote(4);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("shooter command is done", true);
    }

    @Override
    public boolean isFinished() {
        return true;   
    }
}