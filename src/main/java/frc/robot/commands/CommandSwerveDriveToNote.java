package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.CustomGamePieceVision;

import javax.crypto.spec.RC2ParameterSpec;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandSwerveDriveToNote extends Command {    
    private Swerve s_Swerve;
    private CustomGamePieceVision m_CustomGamePieceVision;

    double distanceToNote;

    public CommandSwerveDriveToNote(Swerve s_Swerve, CustomGamePieceVision m_CustomGamePieceVision) {
        this.s_Swerve = s_Swerve;
        this.m_CustomGamePieceVision = m_CustomGamePieceVision;
        addRequirements(s_Swerve);
        addRequirements(m_CustomGamePieceVision);
    }

    @Override
    public void initialize() {
        distanceToNote = m_CustomGamePieceVision.getGamePieceDist();
    }
  
    @Override
    public void end(boolean interrupted) {
        // s_Swerve.drive(
        //     new Translation2d(0.0, 0.0), 
        //     0.0, 
        //     false, 
        //     false
        // );
    }
  
    @Override
    public void execute() {
        /* Drive */
        // s_Swerve.drive(
        //     new Translation2d(0.0, m_CustomGamePieceVision.alignNoteCommands()[1]).times(Constants.Swerve.maxSpeed), 
        //     m_CustomGamePieceVision.alignNoteCommands()[0] * Constants.Swerve.maxAngularVelocity, 
        //     false, 
        //     true
        // );

        s_Swerve.driveToPose(
            new Pose2d(
                new Translation2d(
                    distanceToNote,
                    new Rotation2d(0)
                ), 
                s_Swerve.poseEstimator.getEstimatedPosition().getRotation()
            )
        );
    }
  
    @Override
    public boolean isFinished() {
        // Should return true when no note is seen?
        return Constants.isWithinTol(
            new Translation2d(
                    distanceToNote,
                    new Rotation2d(0)
                ).getX(), 
            s_Swerve.poseEstimator.getEstimatedPosition().getTranslation().getX(), 
            3
        ) && Constants.isWithinTol(
            new Translation2d(
                    distanceToNote,
                    new Rotation2d(0)
                ).getY(), 
            s_Swerve.poseEstimator.getEstimatedPosition().getTranslation().getY(), 
            3
            // Maybe change the tolerance?
        );
    }
}