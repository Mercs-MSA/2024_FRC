package frc.robot.commands.PivotSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.SAT.SAT;

public class CommandPivotScoringPosition extends Command {
    double pivotPos;
    public SAT m_SAT;
    private boolean override;

    public CommandPivotScoringPosition(SAT s){
        m_SAT = s;
        addRequirements(m_SAT);
    }

    // public CommandPivotScoringPosition(SAT s, boolean override, double pos) {
    //     m_SAT = s;
    //     this.override = override;
    //     this.pivotPos = pos;
    //     addRequirements(m_SAT);
    // }


    @Override
    public void initialize() { 

    // if (!override)    
        switch (ScoringConstants.currentScoringMode) {
            case PODIUM:
                pivotPos = SATConstants.PODIUM.pivot;
                break;
            case SUBWOOFER:
                pivotPos = SATConstants.SUBWOOFER.pivot;
                break;
            case AMP1:
                pivotPos = SATConstants.AMP_STAGE_1.pivot;
                break;
            case AMP2: 
                pivotPos = SATConstants.AMP_STAGE_2.pivot;
                break;
            case WING:
                pivotPos = SATConstants.WING.pivot;
                break;
            case TRAP:
                pivotPos = SATConstants.TRAP.pivot;
                break;
        }

        m_SAT.movePivotMotor(pivotPos);
        SmartDashboard.putString("Pivot, I'm trying to go here: ", pivotPos + "");
    }

    @Override
    public void execute() { 

    }

    @Override
    public void end(boolean interupted){
        SmartDashboard.putBoolean("is pivot done?", Constants.isWithinTol(pivotPos, m_SAT.getPivotPos(), SATConstants.MOTOR_TOLERANCE));
    }
    
    @Override
    public boolean isFinished(){
        return (Constants.isWithinTol(pivotPos, m_SAT.getPivotPos(), SATConstants.MOTOR_TOLERANCE));
    }
}    
