package frc.robot.commands.BaseSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.base.Base;

public class CommandBaseScoringPosition extends Command {
    double basePos;
    public Base m_base;

    public CommandBaseScoringPosition(Base m_base) {
        this.m_base = m_base;
        addRequirements(m_base);
    }

    @Override
    public void initialize() {
        switch (ScoringConstants.currentScoringMode) {
            case AMP:
                basePos = SATConstants.AMP_STAGE_1.motor1_base;
                break;
            case TRAP:
                basePos = SATConstants.TRAP.motor1_base;
                break;
            case SUBWOOFER:
                basePos = SATConstants.SUBWOOFER.motor1_base;
                break;
            case WING:
            case PODIUM:
                basePos = SATConstants.START.motor1_base;
                break;
        }

        m_base.moveBaseMotors(basePos); 
        SmartDashboard.putString("base, I'm trying to go here: ", basePos + " ");
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interupted) {
        SmartDashboard.putBoolean("is base done?", 
            Constants.isWithinTol(
                basePos, 
                m_base.getBase1Pos(),
                SATConstants.MOTOR_TOLERANCE
            )
        );
        SmartDashboard.putBoolean("this base command is over", true);
    }

    @Override
    public boolean isFinished() {
        return (
            Constants.isWithinTol(
                basePos, 
                m_base.getBase1Pos(),
                SATConstants.MOTOR_TOLERANCE
            )
        );
    }
}
