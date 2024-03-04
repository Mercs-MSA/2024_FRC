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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class CommandSwerveTurnToNote extends Command {    
    private Swerve s_Swerve;
    private CustomGamePieceVision m_CustomGamePieceVision;
    private double targetYaw, currentYaw;
    private double deltaYaw, finalYaw, currentRotation;
    private PIDController pidController;
    private double rotateVal;


    public CommandSwerveTurnToNote(Swerve s_Swerve, CustomGamePieceVision m_CustomGamePieceVision) {
        this.s_Swerve = s_Swerve;
        this.m_CustomGamePieceVision = m_CustomGamePieceVision;
        pidController = new PIDController(0.2, 0.0, 0.01);
        pidController.setTolerance(1);
        pidController.setSetpoint(0.0);
        addRequirements(s_Swerve);
        addRequirements(m_CustomGamePieceVision);
    }

    @Override
    public void initialize() {
        deltaYaw = 0.513*(m_CustomGamePieceVision.getGamePieceYaw()) + 9.08;
        currentRotation = s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        finalYaw = currentRotation + deltaYaw;
        
        // s_Swerve.driveToPose(new Pose2d(s_Swerve.poseEstimator.getEstimatedPosition().getTranslation().getX() + 0.1, s_Swerve.poseEstimator.getEstimatedPosition().getTranslation().getY(), s_Swerve.poseEstimator.getEstimatedPosition().getRotation())).schedule();
        // s_Swerve.drive(new Translation2d(), Units.degreesToRadians(deltaYaw)*2, true, false);


    }
  
    @Override
    public void end(boolean interrupted) {
        // if (Constants.isWithinTol(finalYaw, s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0.5)){
        //     rotateVal = MathUtil.clamp(pidController.calculate(s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), finalYaw), -5, 5);
        //     s_Swerve.drive(new Translation2d(), rotateVal, true, false);
        // }

        s_Swerve.drive(new Translation2d(), 0, false, false);
    }
  
    @Override
    public void execute() {
        SmartDashboard.putNumber("temp gamePieceYaw", m_CustomGamePieceVision.getGamePieceYaw());
        SmartDashboard.putNumber("temp heading", s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("temp deltaYaw", deltaYaw);
        SmartDashboard.putNumber("temp finalYaw", finalYaw);

        if (m_CustomGamePieceVision.getGamePieceYaw() != 999.0){
            rotateVal = MathUtil.clamp(pidController.calculate(s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), finalYaw), -5, 5);
            s_Swerve.drive(new Translation2d(), rotateVal, true, false);
        }

        // s_Swerve.faceHeading(new Rotation2d(Units.degreesToRadians(deltaYaw)));


    }
  
    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("turn to note is finished", Constants.isWithinTol(finalYaw, s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0.1));
        // Should return true when no note is seen?
        return Constants.isWithinTol(finalYaw, s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0.1);
        // finalYaw == s_Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        // return false;
    }
}