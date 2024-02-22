package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SAT.SAT;

public class CommandBasesPosition extends Command {
    String target;
    double basePos;
    public SAT m_SAT;

    public CommandBasesPosition(String t, SAT s) {
        target = t.toLowerCase();
        m_SAT = s;
        addRequirements(m_SAT);

        switch (target) {
            case "amp":
                basePos = Constants.SATConstants.AMP.motor1_base;
                break;
            case "trap":
                basePos = Constants.SATConstants.TRAP.motor1_base;
                break;
            case "wing":
            case "handoff":
            case "podium":
            case "sub":
            case "start":
            default:
                System.out.println("Invalid Position");
                basePos = Constants.SATConstants.SUBWOOFER.motor1_base;
                break;
        }
    }

    @Override
    public void initialize() {
        m_SAT.moveBaseMotors(basePos); 
        SmartDashboard.putString("base, I'm trying to go here: ", basePos + " ");
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interupted) {
        SmartDashboard.putBoolean("is base done?", 
            isWithinTol(
                basePos, 
                m_SAT.getBase1Pos(),
                Constants.SATConstants.MOTOR_TOLERANCE
            )
        );
    }

    @Override
    public boolean isFinished() {
        return (
            isWithinTol(
                basePos, 
                m_SAT.getBase1Pos(),
                Constants.SATConstants.MOTOR_TOLERANCE
            )
        );
    }

    public boolean isWithinTol(double targetPose, double currentPose, double tolerance) {
        return (Math.abs(targetPose - currentPose) <= tolerance);
    }
}
