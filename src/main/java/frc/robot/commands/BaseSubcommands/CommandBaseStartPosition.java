package frc.robot.commands.BaseSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SAT.SAT;

public class CommandBaseStartPosition extends Command {
    double basePos = Constants.SATConstants.START.motor1_base;
    public SAT m_SAT;

    public CommandBaseStartPosition(SAT s) {
        m_SAT = s;
        addRequirements(m_SAT);
    }

    @Override
    public void initialize() {
        m_SAT.moveBaseMotors(basePos); 
        SmartDashboard.putString("base, I'm trying to go here: ", basePos + " ");
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interupted) {
        m_SAT.neturalBase();
        SmartDashboard.putBoolean("is base done?", 
        Constants.isWithinTol(
                basePos, 
                m_SAT.getBase1Pos(),
                Constants.SATConstants.MOTOR_TOLERANCE
            )
        );
    }

    @Override
    public boolean isFinished() {
        return (
            Constants.isWithinTol(
                basePos, 
                m_SAT.getBase1Pos(),
                Constants.SATConstants.MOTOR_TOLERANCE
            )
        );
    }
}
