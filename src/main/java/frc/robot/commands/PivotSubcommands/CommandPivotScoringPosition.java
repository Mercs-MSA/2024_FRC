package frc.robot.commands.PivotSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.pivot.Pivot;

public class CommandPivotScoringPosition extends Command {
    double pivotPos;
    public Pivot m_pivot;

    public CommandPivotScoringPosition(Pivot s){
        m_pivot = s;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize() { 
        switch (ScoringConstants.currentScoringMode) {
            case PODIUM:
                pivotPos = SATConstants.PODIUM.pivot;
                break;
            case SUBWOOFER:
                pivotPos = SATConstants.SUBWOOFER.pivot;
                break;
            case AMP:
                pivotPos = SATConstants.AMP_STAGE_1.pivot;
                break;
            case WING:
                pivotPos = SATConstants.WING.pivot;
                break;
            case TRAP:
                pivotPos = SATConstants.TRAP.pivot;
                break;
        }

        m_pivot.movePivotMotor(pivotPos);
        SmartDashboard.putString("Pivot, I'm trying to go here: ", pivotPos + "");
    }

    @Override
    public void execute() { 

    }

    @Override
    public void end(boolean interupted){
        SmartDashboard.putBoolean("is pivot done?", Constants.isWithinTol(pivotPos, m_pivot.getPivotPos(), SATConstants.MOTOR_TOLERANCE));
    }
    
    @Override
    public boolean isFinished(){
        return (Constants.isWithinTol(pivotPos, m_pivot.getPivotPos(), SATConstants.MOTOR_TOLERANCE));
    }
}    
