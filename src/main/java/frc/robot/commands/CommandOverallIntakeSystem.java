package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SAT.SAT;
import frc.robot.subsystems.intake.Intake;

public class CommandOverallIntakeSystem extends Command{
    private final Intake m_intake;
    private final SAT m_SAT;

    public CommandOverallIntakeSystem(Intake m_intake, SAT m_SAT){
        this.m_intake = m_intake;
        this.m_SAT = m_SAT;
        addRequirements(this.m_intake, this.m_SAT);
    }

    @Override
    public void initialize() {
        if (!(m_intake.upperSensorDetectsNote() || m_intake.lowerSensorDetectsNote())){ //if no note
            m_intake.enableIntakeLowerSensorInterrupt();
            m_intake.startIntakeMotor();
        }
    }

    @Override
    public void execute() {
        if (m_intake.lowerSensorDetectsNote()){
            m_intake.disableIntakeLowerSensorInterrupt();
            m_intake.enableIntakeUpperSensorInterrupt();
            m_SAT.movePivotMotor("handoff");
            if (m_SAT.isWithinTol(Constants.SATConstants.HANDOFF.pivot, m_SAT.getPivotPos(), 0.01)){
                m_intake.startIndexMotor();
            }
            else {
                m_intake.intakeMotorToPosition(0);//move note further inside intake by moving intake a specific amount
            }
        }
        else if (m_intake.upperSensorDetectsNote()){
            m_intake.disableIntakeUpperSensorInterrupt();
            m_intake.stopIntakeIndexerMotors();
            m_intake.indexMotorToPosition(0); //move index a specific amount forward 
            m_SAT.movePivotMotor("start");      
        }
        else {
            if (!(m_intake.upperSensorDetectsNote() || m_intake.lowerSensorDetectsNote())){ //if no note
                m_intake.enableIntakeLowerSensorInterrupt();
                m_intake.startIntakeMotor();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        //change state to ready to shoot
    }

    @Override
    public boolean isFinished() {
        return false; //chnage to manual mode state
    }
}
