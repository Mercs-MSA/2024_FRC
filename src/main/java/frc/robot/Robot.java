// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import com.ctre.phoenix6.signals.System_StateValue;

import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.State.robotState;
// import frc.robot.Constants.Vision.aprilTagBackLeft;
import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
      SmartDashboard.putString("STATUS CODE Description", "");
      SmartDashboard.putBoolean("STATUS CODE IS WARNING?", false);
      SmartDashboard.putString("STATUS CODE", "");

    // Optional<Alliance> alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    //   Constants.Vision.isRedAlliance = (alliance.get() == DriverStation.Alliance.Red);
    // }
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
    SmartDashboard.putString("Pose", RobotContainer.s_Swerve.getPose().toString());
    SmartDashboard.putBoolean("Robot Has Note", Constants.IntakeConstants.kRobotHasNote);

    SmartDashboard.putNumber("LeftY", -m_robotContainer.driver.getLeftY());
    SmartDashboard.putNumber("LeftX", -m_robotContainer.driver.getLeftX());
    SmartDashboard.putNumber("RightX", -m_robotContainer.driver.getRightX());
    SmartDashboard.putNumber("works?", m_robotContainer.m_SAT.works);

    

    if (m_robotContainer.s_Swerve.getCurrentCommand() != null) {
      SmartDashboard.putString("SwerveCommand", m_robotContainer.s_Swerve.getCurrentCommand().getName());
    }
    SmartDashboard.putBoolean("Current A State", m_robotContainer.driver.getRawButton(2));
    
      



    SmartDashboard.putNumber("gyro yaw", m_robotContainer.s_Swerve.gyro.getAngle());

    SmartDashboard.putBoolean("Are we red alliance?", Constants.Vision.isRedAlliance);

    // if (m_robotContainer.driver.getAButton()) {
    if (m_robotContainer.driver.getCrossButton()) {
      // Constants.State.setState("PIVOT");
      StatusCode status = m_robotContainer.m_SAT.movePivot(-12.0);
      SmartDashboard.putString("STATUS CODE Description", status.getDescription());
      SmartDashboard.putBoolean("STATUS CODE IS WARNING?", status.isError());
      SmartDashboard.putString("STATUS CODE", status.getName());
    }
    else if (m_robotContainer.driver.getTriangleButton()) {
      m_robotContainer.m_SAT.movePivot(-8.0);
    }
    if (m_robotContainer.driver.getSquareButton()) {
          m_robotContainer.m_Intake.stopIntakeMotor();
    }

    // if (m_robotContainer.driver.getRightTriggerAxis() > 0.10)
    // {
      
    // }
    if (m_robotContainer.driver.getR1Button()) {
      m_robotContainer.m_SAT.startShooter(0.2);
    } else if (m_robotContainer.driver.getL1Button()) {
      m_robotContainer.m_SAT.stopShooter();
    }
    // SmartDashboard.putNumber("PIVOT POS", kDefaultPeriod)

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
    //SmartDashboard.putNumber("MiniPC Input Voltage (volts)", Constants.Misc.Conversion_Factor*PSU_Volt_Monitor.getAverageVoltage());

    // poseEstimateField2d.setRobotPose(RobotContainer.s_Swerve.poseEstimator.getEstimatedPosition());
    // SmartDashboard.putData("estimated robot pose", poseEstimateField2d);

    SmartDashboard.putData(CommandScheduler.getInstance());

    
    SmartDashboard.putString("Scoring Mode", Constants.ScoringConstants.currentScoringMode.toString());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.m_SAT.movePivot(-1.21);
    resetAllMotorCommands();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
    resetAllMotorCommands();
    // Constants.Vision.visionTurnedOn = true;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    resetAllMotorCommands();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


  /** This function makes sure that all outstanding commands are cleared and that all motors are commanded to stop.
   ** This should be run whenever you want to make sure that everything stops being controlled. */  
  public void resetAllMotorCommands() {
    CommandScheduler.getInstance().cancelAll();
  }

  public void goToHomePos(){}

}