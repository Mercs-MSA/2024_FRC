package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.CustomGamePieceVision;

import javax.crypto.spec.RC2ParameterSpec;
import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandSwerveTurnToNote extends Command {    
    private Swerve s_Swerve;
    private CustomGamePieceVision m_CustomGamePieceVision;
    private double targetYaw, currentYaw;
    private double deltaYaw, finalYaw;

    public CommandSwerveTurnToNote(Swerve s_Swerve, CustomGamePieceVision m_CustomGamePieceVision) {
        this.s_Swerve = s_Swerve;
        this.m_CustomGamePieceVision = m_CustomGamePieceVision;
        addRequirements(s_Swerve);
        addRequirements(m_CustomGamePieceVision);
    }

    @Override
    public void initialize() {
        deltaYaw = 0.501*(m_CustomGamePieceVision.getGamePieceYaw()) + 11.3;
        finalYaw = s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees() + deltaYaw;

        s_Swerve.drive(new Translation2d(), finalYaw, true, true);

        // targetYaw = s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees() + (m_CustomGamePieceVision.getGamePieceYaw() - Constants.Vision.gamePieceYawOffset);
        // currentYaw = s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        if (Math.abs(m_CustomGamePieceVision.getGamePieceYaw()) != 999.0){
            if (Math.abs(m_CustomGamePieceVision.getGamePieceYaw() - Constants.Vision.gamePieceYawOffset) > 10){
                s_Swerve.drive(new Translation2d(), (targetYaw - currentYaw)/20, false, false);
            }
            else if (Math.abs(m_CustomGamePieceVision.getGamePieceYaw() - Constants.Vision.gamePieceYawOffset) > 5){
                s_Swerve.drive(new Translation2d(), (targetYaw - currentYaw)/10, false, false);
            }
            else if (Math.abs(m_CustomGamePieceVision.getGamePieceYaw() - Constants.Vision.gamePieceYawOffset) > 0){
                s_Swerve.drive(new Translation2d(), (targetYaw - currentYaw)/5, false, false);
            }
            else {
                s_Swerve.drive(new Translation2d(), 0, false, false);

            }
        }
    }
  
    @Override
    public void end(boolean interrupted) {
        // s_Swerve.drive(new Translation2d(), 0, false, false);
    }
  
    @Override
    public void execute() {
        SmartDashboard.putNumber("temp gamePieceYaw", m_CustomGamePieceVision.getGamePieceYaw());
        SmartDashboard.putNumber("temp heading", s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("temp deltaYaw", deltaYaw);
        SmartDashboard.putNumber("temp finalYaw", finalYaw);
        /* Drive */
        // s_Swerve.drive(
        //     new Translation2d(0.0, m_CustomGamePieceVision.alignNoteCommands()[1]).times(Constants.Swerve.maxSpeed), 
        //     m_CustomGamePieceVision.alignNoteCommands()[0] * Constants.Swerve.maxAngularVelocity, 
        //     false, 
        //     true
        // );

        // s_Swerve.driveToPose(
        //     new Pose2d(
        //         new Translation2d(s_Swerve.poseEstimator.getEstimatedPosition().getTranslation().getX()-0.01, s_Swerve.poseEstimator.getEstimatedPosition().getTranslation().getY()),
        //         new Rotation2d(
        //             Units.degreesToRadians(
        //                 s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees()+(m_CustomGamePieceVision.getGamePieceYaw() - Constants.Vision.gamePieceYawOffset)
        //             )
        //         )
        //     )
        // );


    }
  
    @Override
    public boolean isFinished() {
        // Should return true when no note is seen?
        return Constants.isWithinTol(
            targetYaw,
            s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 
            1
        );
    }
}