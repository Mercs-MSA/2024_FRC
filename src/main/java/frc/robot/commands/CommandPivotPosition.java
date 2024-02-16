package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SAT.SAT;

public class CommandPivotPosition extends Command {
    double target;
    public SAT m_SAT;

    public CommandPivotPosition(double t, SAT s){
        target = t;
        m_SAT = s;
        addRequirements(m_SAT);
    }

    @Override
    public void initialize(){
        m_SAT.movePivotMotor(target);
        SmartDashboard.putString("Pivot, I'm trying to go here: ", target + "");
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interupted){
        SmartDashboard.putBoolean("is pivot done?", isWithinTol(target, m_SAT.getPivotPos(), Constants.SATConstants.MOTOR_TOLERANCE));
    }

    @Override
    public boolean isFinished(){
        return (isWithinTol(target, m_SAT.getPivotPos(), Constants.SATConstants.MOTOR_TOLERANCE));
    }

    public boolean isWithinTol(double targetPose, double currentPose, double tolerance){
        return (Math.abs(targetPose - currentPose) <= tolerance);
    }
}




