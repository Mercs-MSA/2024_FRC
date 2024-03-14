package frc.robot.commands.PivotSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.pivot.Pivot;

public class CommandPivotStartPosition extends Command {
    double pivotPos = Constants.SATConstants.START.pivot;
    public Pivot m_pivot;

    public CommandPivotStartPosition(Pivot s){
        m_pivot = s;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize(){
        m_pivot.movePivotMotor(pivotPos);
        SmartDashboard.putString("Pivot, I'm trying to go here: ", pivotPos + "");
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interupted){
        SmartDashboard.putBoolean("is pivot done?", Constants.isWithinTol(pivotPos, m_pivot.getPivotPos(), Constants.SATConstants.MOTOR_TOLERANCE));
    }

    @Override
    public boolean isFinished(){
        return (Constants.isWithinTol(pivotPos, m_pivot.getPivotPos(), Constants.SATConstants.MOTOR_TOLERANCE));
    }
}




