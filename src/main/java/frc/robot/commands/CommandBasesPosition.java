package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SAT.SAT;

public class CommandBasesPosition extends Command{

    double target1;
    double target2;
    public SAT m_SAT;

    public CommandBasesPosition(double t1, double t2, SAT s){

        target1 = t1;
        target2 = t2;
        m_SAT = s;
        addRequirements(m_SAT);


    }

    @Override
    public void initialize(){
        m_SAT.moveBaseMotors(target1, target2); 
        SmartDashboard.putString("base, I'm trying to go here: ", target1 + " " + target2);

    }

    @Override
    public void execute(){

    }


    @Override
    public void end(boolean interupted){
        SmartDashboard.putBoolean("is base done?", isWithinTol(target1, m_SAT.getBase1Pos(), 0.1) && isWithinTol(target2, m_SAT.getBase2Pos(), 0.1));
    }


    @Override
    public boolean isFinished(){
        return (isWithinTol(target1, m_SAT.getBase1Pos(), 0.1) && isWithinTol(target2, m_SAT.getBase2Pos(), 0.1));

    }
    public boolean isWithinTol(double targetPose, double currentPose, double tolerance){
        return (Math.abs(targetPose - currentPose) <= tolerance);
    }

    


}
