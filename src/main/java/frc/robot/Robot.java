// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.State.robotState;
import frc.robot.subsystems.vision.ApriltagVision;
import frc.robot.subsystems.climber.climber;
import frc.robot.subsystems.SAT.SAT;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer = new RobotContainer();

  robotState currentRobotState = robotState.IDLE;
  // ApriltagVision m_ApriltagVision = new ApriltagVision("apriltag2");
  Field2d poseEstimateField2d = new Field2d();
  Pose2d apiltagPlusGyro = new Pose2d();
  private AnalogInput PSU_Volt_Monitor = new AnalogInput(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer.configureButtonBindings();
    Constants.State.setState("IDLE");
    // m_ApriltagVision.periodic();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putString("Current Robot State", Constants.State.getState().toString());
    SmartDashboard.putString("Pose", m_robotContainer.s_Swerve.getPose().toString());
    // SmartDashboard.putNumber("Climber Left motor Pos: ", m_robotContainer.m_climber.outputLeftData());
    // SmartDashboard.putNumber("Climber Right motor Pos: ", m_robotContainer.m_climber.outputRightData());
    // SmartDashboard.putNumber("Base1 Pos", m_robotContainer.m_SAT.outputBase1Data());
    // SmartDashboard.putNumber("Base2 Pos", m_robotContainer.m_SAT.outputBase2Data());
    // SmartDashboard.putNumber("Pivot Pos", m_robotContainer.m_SAT.outputPivotData());
    // SmartDashboard.putNumber("Climber Left motor Pos: ", m_robotContainer.m_climber.outputLeftData());
    // SmartDashboard.putNumber("Climber Right motor Pos: ", m_robotContainer.m_climber.outputRightData());
    // SmartDashboard.putNumber("Base1 Pos", m_robotContainer.m_SAT.outputBase1Data());
    // SmartDashboard.putNumber("Base2 Pos", m_robotContainer.m_SAT.outputBase2Data());
    // SmartDashboard.putNumber("Pivot Pos", m_robotContainer.m_SAT.outputPivotData());
    SmartDashboard.putNumber("MiniPC Input Voltage (volts)", Constants.Misc.Conversion_Factor*PSU_Volt_Monitor.getAverageVoltage());
    // //m_ApriltagVision.periodic();

    // if (m_ApriltagVision.hasMultiTagEstimatedPose()){ //replace with hasTargets()?
    //   apiltagPlusGyro = new Pose2d(new Translation2d(m_ApriltagVision.getGlobalPoseEstimate().getTranslation().getX() - 0.29, m_ApriltagVision.getGlobalPoseEstimate().getTranslation().getY()), m_robotContainer.s_Swerve.getPose().getRotation());
    //   m_robotContainer.s_Swerve.poseEstimator.addVisionMeasurement(apiltagPlusGyro, m_ApriltagVision.getTimestampSeconds());
    // }
    poseEstimateField2d.setRobotPose(m_robotContainer.s_Swerve.poseEstimator.getEstimatedPosition());
    SmartDashboard.putData("estimated robot pose", poseEstimateField2d);
    SmartDashboard.putNumber("Current Heading", m_robotContainer.s_Swerve.getHeading().getRadians());
    SmartDashboard.putData("Auto Mode", m_robotContainer.autoChooser);

    // SmartDashboard.putData(m_robotContainer.m_intake);
    // SmartDashboard.putData(m_robotContainer.m_SAT);
  //  SmartDashboard.putData(m_robotContainer.m_climber);
    // SmartDashboard.putData(m_robotContainer.m_SAT);
    SmartDashboard.putData(m_robotContainer.s_Swerve);
    SmartDashboard.putData(CommandScheduler.getInstance());

   // SmartDashboard.putBoolean("intake ir bool", m_robotContainer.m_intake.detectNote());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
