package frc.robot.commands.PivotSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SAT.SAT;

public class CommandPivotHandoffPosition extends Command {
    double pivotPos = Constants.SATConstants.HANDOFF.pivot;
    public SAT m_SAT;

    public CommandPivotHandoffPosition(SAT s){
        m_SAT = s;
        addRequirements(m_SAT);
    }

    @Override
    public void initialize(){
        m_SAT.movePivotMotor(pivotPos);
        SmartDashboard.putString("Pivot, I'm trying to go here: ", pivotPos + "");
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interupted){
        m_SAT.neturalPivot();
        SmartDashboard.putBoolean("is pivot done?", Constants.isWithinTol(pivotPos, m_SAT.getPivotPos(), Constants.SATConstants.MOTOR_TOLERANCE));
    }

    @Override
    public boolean isFinished(){
        m_SAT.neturalPivot();
        return (Constants.isWithinTol(pivotPos, m_SAT.getPivotPos(), Constants.SATConstants.MOTOR_TOLERANCE));
    }

}




