package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandSwerveDriveToNote extends Command {    
    private Swerve s_Swerve;
    private Intake m_intake;

    double distanceToNote;

    public CommandSwerveDriveToNote(Swerve s_Swerve, Intake m_intake) {
        this.s_Swerve = s_Swerve;
        this.m_intake = m_intake;
        addRequirements(s_Swerve);
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        // distanceToNote = m_CustomGamePieceVision.getGamePieceDist();
        // m_intake.startIndexMotor();
    }
  
    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, 0), 0, false, false);
        // m_intake.stopIntakeMotor();
    }
  
    @Override
    public void execute() {
        s_Swerve.drive(new Translation2d(3.5, 0), 0, false, false);
    }
  
    @Override
    public boolean isFinished() {
        // Should return true when no note is seen?
        return m_intake.lowerSensorDetectsNote();
    }
}