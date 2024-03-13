package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class CommandDriveToPose extends Command {

  private final Swerve swerve;
  private Pose2d desiredPose;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(3, Constants.Swerve.driveKI, Constants.Swerve.driveKD, new TrapezoidProfile.Constraints(3.5, 3));
  private final ProfiledPIDController yController =
      new ProfiledPIDController(3, Constants.Swerve.driveKI, Constants.Swerve.driveKD, new TrapezoidProfile.Constraints(3.5, 3));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD, new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));

  public CommandDriveToPose(Swerve swerve, Pose2d pose) {
    this.swerve = swerve;
    this.desiredPose = pose;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    thetaController.setTolerance(Units.degreesToRadians(1));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    var currPose = swerve.getPose();
    xController.reset(currPose.getX());
    yController.reset(currPose.getY());
    thetaController.reset(currPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    var currPose = swerve.getPose();
    var targetPose = desiredPose;

    double xvelocity = xController.calculate(currPose.getX(), targetPose.getX());
    double yvelocity = yController.calculate(currPose.getY(), targetPose.getY());
    double thetaVelocity =
        thetaController.calculate(
            currPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    if (atGoal()) {
      xvelocity = 0.0;
      yvelocity = 0.0;
      thetaVelocity = 0.0;
    }

    swerve.driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xvelocity, yvelocity, thetaVelocity, currPose.getRotation()));
  }

  public boolean atGoal() {
    return (xController.atGoal() && yController.atGoal() && thetaController.atGoal());
  }

  @Override
  public boolean isFinished(){
    return atGoal();
  }
}