package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SAT.SAT;

public class CommandBasesPosition extends Command{

    String target;
    public SAT m_SAT;

    public CommandBasesPosition(String t, SAT s){

        target = t;
        m_SAT = s;
        addRequirements(m_SAT);


    }

    @Override
    public void initialize(){

        if (target == "Podium"){
            m_SAT.goToBasePodiumPosition();

        }
        else if (target == "Sub"){
            m_SAT.goToBaseSubPosition();

        }
        else if (target == "Amp"){
            m_SAT.goToBaseAmpPosition();

        }
        else if (target == "Trap"){
            m_SAT.goToBaseTrapPosition();

        }
        else if (target == "Zero"){
            m_SAT.goToBaseZeroPosition();

        }
        else if (target == "Wing"){


        }


        
    }

    @Override
    public void execute(){

    }


    @Override
    public void end(boolean interupted){

    }


    @Override
    public boolean isFinished(){
        if (target == "Podium"){
            return isWithinTol(Constants.SATConstants.BASE_PODIUM_POS, m_SAT.getBase1Pos(), 0.3);
        }
        else if (target == "Sub"){
            return isWithinTol(Constants.SATConstants.BASE_SUB_POS, m_SAT.getBase1Pos(), 0.3);

        }
        else if (target == "Amp"){
            return isWithinTol(Constants.SATConstants.BASE_AMP_POS, m_SAT.getBase1Pos(), 0.3);

        }
        else if (target == "Trap"){
         return isWithinTol(Constants.SATConstants.BASE_TRAP_POS, m_SAT.getBase1Pos(), 0.3);

        }
        else if (target == "Zero"){
         return isWithinTol(0, m_SAT.getBase1Pos(), 0.3);

        }
        else {
            return false;

        }

    }

    public boolean isWithinTol(double targetPose, double currentPose, double tolerance){
        if (Math.abs(targetPose - currentPose) <= tolerance){
            return true;
        }
        else {
            return false;
        }
    }

    


}
