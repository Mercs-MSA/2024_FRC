package frc.robot.commands.BaseSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.base.Base;

public class CommandBaseStageTwoPosition extends Command {
    double basePos = Constants.SATConstants.AMP_STAGE_2.motor1_base;
    public Base m_base;

    public CommandBaseStageTwoPosition(Base s){
        m_base = s;
        addRequirements(m_base);
    }

    @Override
    public void initialize(){
        m_base.moveBaseMotors(basePos);
        SmartDashboard.putString("Base, I'm trying to go here: ", basePos + "");
    }

    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interupted){
        SmartDashboard.putBoolean("is base done?", Constants.isWithinTol(basePos, m_base.getBase1Pos(), Constants.SATConstants.MOTOR_TOLERANCE));
    }

    @Override
    public boolean isFinished(){
        return (Constants.isWithinTol(basePos, m_base.getBase1Pos(), Constants.SATConstants.MOTOR_TOLERANCE));
    }
}




