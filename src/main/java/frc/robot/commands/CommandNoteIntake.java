package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.CustomGamePieceVision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class CommandNoteIntake extends Command {    
    private Swerve s_Swerve;
    private Intake m_intake;
    private CustomGamePieceVision m_CustomGamePieceVision;

    public CommandNoteIntake(Swerve s_Swerve, Intake m_intake, CustomGamePieceVision m_CustomGamePieceVision) {
        this.s_Swerve = s_Swerve;
        this.m_intake = m_intake;
        this.m_CustomGamePieceVision = m_CustomGamePieceVision;
        addRequirements(s_Swerve);
        addRequirements(m_intake);
        addRequirements(m_CustomGamePieceVision);
    }

    @Override
    public void initialize() {
    }
  
    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(
            new Translation2d(0.0, 0.0), 
            0.0, 
            false, 
            true
        );
        m_intake.stopIntakeMotor();
    }
  
    @Override
    public void execute() {
        /* Drive */
        s_Swerve.drive(
            new Translation2d(0.0, m_CustomGamePieceVision.alignNoteCommands()[1]).times(Constants.Swerve.maxSpeed), 
            m_CustomGamePieceVision.alignNoteCommands()[0] * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
        m_intake.feedToShooter();

    }
  
    @Override
    public boolean isFinished() {
      return m_intake.detectNote() == false;
    }
}