package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SAT.SAT;





public class CommandBasesPosition extends Command{

    String target;
    public SAT m_SAT = new SAT();

    public CommandBasesPosition(String t){

        target = t;


    }

    @Override
    public void initialize(){

        if (target == "Podium"){
            m_SAT.goToBasePodiumPosition();

        }
        else if (target == "Sub"){


        }
        else if (target == "Amp"){


        }
        else if (target == "Trap"){


        }
        else if (target == "Zero"){


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
        return false;


    }


}
