package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.SAT.SAT;

public class CommandShooterStart extends Command {
    double shooterSpeed;
    public SAT m_SAT;

    public CommandShooterStart(SAT s) {
        m_SAT = s;
        addRequirements(m_SAT);
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

        m_SAT.shootNote(shooterSpeed);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_SAT.getShooterSpeed() - shooterSpeed) <= SATConstants.kShooterSpeedTolerance;
    }

}