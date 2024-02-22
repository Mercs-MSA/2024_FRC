package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants.*;
import frc.robot.subsystems.SAT.SAT;

public class CommandMovePivotToPosition extends Command {
    ScoringMode target;
    double pivotPos;
    public SAT m_SAT;

    public CommandMovePivotToPosition(SAT s){
        this(s, ScoringConstants.currentScoringMode);
    }

    public CommandMovePivotToPosition(SAT s, ScoringMode t){
        target = t;
        m_SAT = s;
        addRequirements(m_SAT);

        switch (target) {
            case PODIUM:
                pivotPos = Constants.SATConstants.PODIUM.pivot;
                break;
            case SUB:
                pivotPos = Constants.SATConstants.SUBWOOFER.pivot;
                break;
            case AMP:
                pivotPos = Constants.SATConstants.AMP.pivot;
                break;
            case WING:
                pivotPos = Constants.SATConstants.WING.pivot;
                break;
            case TRAP:
                pivotPos = Constants.SATConstants.TRAP.pivot;
                break;
        }
    }

    @Override
    public void initialize() { 
        m_SAT.movePivotMotor(pivotPos);
        SmartDashboard.putString("Pivot, I'm trying to go here: ", pivotPos + "");
    }

    @Override
    public void execute() { 
    }

    @Override
    public void end(boolean interupted){
        SmartDashboard.putBoolean("is pivot done?", isWithinTol(pivotPos, m_SAT.getPivotPos(), Constants.SATConstants.MOTOR_TOLERANCE));
    }
    
    @Override
    public boolean isFinished(){
        return (isWithinTol(pivotPos, m_SAT.getPivotPos(), Constants.SATConstants.MOTOR_TOLERANCE));
    }

    public boolean isWithinTol(double targetPose, double currentPose, double tolerance){
        return (Math.abs(targetPose - currentPose) <= tolerance);
    }
}    
