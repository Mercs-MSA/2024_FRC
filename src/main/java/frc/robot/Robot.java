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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants.indexState;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.State.robotState;
// import frc.robot.Constants.Vision.aprilTagBackLeft;
import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DigitalInput;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private Command m_autonomousCommand;
  private enum ShootingState {
    IDLE,
    INTAKING,
    LOADED,
    EJECT
  }

  private enum PivotState {
    BASE_SUBWOOFER(0, 1),
    EJECT(-27, 0),
    PODIUM(2.0, 0),
    FREE_MOVE(2.0, 0);

    private double position;
    private int slot;

    PivotState(double position, int slot)
    {
      this.position = position;
      this.slot = slot;
    }

    public double getPos()
    {
      return position;
    }
    public int getSlot()
    {
      return slot;
    }
  }

  private ShootingState currentShootState = ShootingState.IDLE;
  private PivotState currentPivotState = PivotState.BASE_SUBWOOFER;
  private Timer timer = new Timer();
  private final double shooterThreshold = 6.5;
  private boolean hasShooterStarted = false;
  private boolean orgShooterStarted = false;
  private final RobotContainer m_robotContainer = new RobotContainer();
  private int stage = 0;
  robotState currentRobotState = robotState.IDLE;

  Field2d poseEstimateField2d = new Field2d();
  Pose2d apiltagPlusGyro = new Pose2d();
  private AnalogInput PSU_Volt_Monitor = new AnalogInput(0);
  private DigitalInput beamDigitalInput = new DigitalInput(1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    timer.restart();
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
    m_robotContainer.m_SAT.setStartPivotPos(0.0);
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
    if (!orgShooterStarted)
    {
      m_robotContainer.m_SAT.startShooter(shooterThreshold);
      hasShooterStarted = true;
      orgShooterStarted = true;
    }
    if (m_robotContainer.driver.getRightStickButtonPressed()) {
      if (hasShooterStarted)
      {
        m_robotContainer.m_SAT.stopShooter();
        hasShooterStarted = false;
      }
      else 
      {
        m_robotContainer.m_SAT.startShooter(shooterThreshold);
        hasShooterStarted = true;
      }
    }
    
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
    
   SmartDashboard.putBoolean("ROBOT CLASS BEAM BREAK: ", beamDigitalInput.get());

    SmartDashboard.putNumber("gyro yaw", m_robotContainer.s_Swerve.gyro.getAngle());

    SmartDashboard.putBoolean("Are we red alliance?", Constants.Vision.isRedAlliance);

    SmartDashboard.putNumber("pivot pos:", m_robotContainer.m_SAT.getPivotPos());

    switch (currentShootState)
    {
      case IDLE:
        if (m_robotContainer.m_SAT.pivotInRange(PivotState.BASE_SUBWOOFER.getPos()))
        {
          if (m_robotContainer.driver.getLeftBumperPressed())
          {
            m_robotContainer.m_Intake.runIntakeIndex(8);
            currentShootState = ShootingState.INTAKING;
          }
          if (!beamDigitalInput.get()) {
            currentShootState = ShootingState.LOADED;
            
          }
        }
        break;
      case INTAKING:
        if (m_robotContainer.driver.getLeftBumperPressed()) {
          m_robotContainer.m_Intake.resetMotors();
          currentShootState = ShootingState.IDLE;
          break;
        }
        if (!beamDigitalInput.get())
        {
          m_robotContainer.m_Intake.resetMotors();
          currentShootState = ShootingState.LOADED;
          m_robotContainer.driver.setRumble(RumbleType.kBothRumble, 100);
          timer.start();
        }
        break;
      case LOADED: 
        if (m_robotContainer.driver.getRightBumper())
        {
          m_robotContainer.m_Intake.startIndexMotor();
        }
        if (beamDigitalInput.get()) {
          m_robotContainer.m_Intake.stopIndexMotor();
          currentShootState = ShootingState.IDLE;
        }
        if (m_robotContainer.driver.getBButtonPressed()) {
          currentPivotState = PivotState.EJECT;
          
          // m_robotContainer.m_SAT.movePivot(currentPivotState.getPos());
          currentShootState = ShootingState.EJECT;
        }
        if (m_robotContainer.driver.getPOV() == 270)
        {
          currentPivotState = PivotState.PODIUM;
        }
        break;
      case EJECT:
        if (m_robotContainer.m_SAT.pivotInRange(PivotState.EJECT.getPos()))
        {
          if (m_robotContainer.driver.getBButtonPressed())
          {
            m_robotContainer.m_Intake.reverseIndexMotor();
          }
          if (m_robotContainer.driver.getAButtonPressed())
          {
            m_robotContainer.m_Intake.stopIndexMotor();
            currentPivotState = PivotState.BASE_SUBWOOFER;
            currentShootState = ShootingState.IDLE;
          }
        }
        break;
      default:
        SmartDashboard.putString("What's going on?", "?????");
        m_robotContainer.m_SAT.stopShooter();
        m_robotContainer.m_Intake.resetMotors();
        hasShooterStarted = false;
    }
  
    switch (currentPivotState)
    {
      case BASE_SUBWOOFER:
        m_robotContainer.m_SAT.movePivot(currentPivotState.getPos(), currentPivotState.getSlot());
        break;
      default:
        m_robotContainer.m_SAT.movePivot(currentPivotState.getPos(), 0);
        break;
    }
    
  

    if (m_robotContainer.driver.getStartButton())
    {
      m_robotContainer.m_SAT.stopShooter();
      m_robotContainer.m_Intake.resetMotors();
      hasShooterStarted = false;
      stage = 0;
    }
    if (timer.get() > 2)
    {
      timer.restart();
      m_robotContainer.driver.setRumble(RumbleType.kBothRumble, 0);
    }
    SmartDashboard.putString("Shooting Stage:", currentShootState.toString());
    SmartDashboard.putString("Pivot Stage: ", currentPivotState.toString());
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
    m_robotContainer.m_SAT.movePivot(0, 1);
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
    m_robotContainer.m_SAT.changeToCoast();
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
  public void control(int control)
  {
    
  }
  public void goToHomePos(){}

}