package frc.robot.commands.PivotSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SAT.SAT;

public class CommandPivotStageTwoPosition extends Command {
    double pivotPos = Constants.SATConstants.AMP_STAGE_2.pivot;
    public SAT m_SAT;

    public CommandPivotStageTwoPosition(SAT s){
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




