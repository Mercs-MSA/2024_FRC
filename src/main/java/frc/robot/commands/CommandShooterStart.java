package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.shooter.Shooter;

public class CommandShooterStart extends Command {
    double shooterSpeed;
    public Shooter m_shooter;

    public CommandShooterStart(Shooter s) {
        m_shooter = s;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        switch (ScoringConstants.currentScoringMode) {
            case PODIUM:
                shooterSpeed = SATConstants.PODIUM.shooterSpeed;
                break;
            case SUBWOOFER:
                shooterSpeed = SATConstants.SUBWOOFER.shooterSpeed;
                break;
            case AMP:
                shooterSpeed = SATConstants.AMP_STAGE_2.shooterSpeed;
                break;
            case WING:
                shooterSpeed = SATConstants.WING.shooterSpeed;
                break;
            case TRAP:
                shooterSpeed = SATConstants.TRAP.shooterSpeed;
                break;
        }

        m_shooter.shootNote(shooterSpeed);
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
        return Math.abs(m_shooter.getShooterSpeed() - shooterSpeed) <= SATConstants.kShooterSpeedTolerance;
        
    }

}