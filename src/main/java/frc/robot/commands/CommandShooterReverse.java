package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SAT.SAT;

public class CommandShooterReverse extends Command {
    public SAT m_SAT;

    public CommandShooterReverse(SAT s) {
        m_SAT = s;
        addRequirements(m_SAT);
    }

    @Override
    public void initialize() {
        m_SAT.shootNote(4);
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