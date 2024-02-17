// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SATConstants;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.IntakeSubcommands.*;
import frc.robot.commands.IndexSubcommands.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;

//  USE NEXT LINE FOR TESTING
import frc.robot.sim.PhysicsSim;

public class Intake extends SubsystemBase {
  // USE NEXT LINE FOR TESTING
  public boolean simulationDebugMode = false;

  private boolean isUpperNotePresent;
  private boolean isLowerNotePresent;
  private boolean isNEWSENSORPresent;
  private boolean isNEWSENSOR2Present;

  
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorId); //carpet
  private final TalonFX indexMotor = new TalonFX(IntakeConstants.kIndexMotorId); //sat (feeder)
  private final DutyCycleOut intakeMotor_dutyCycleOut = new DutyCycleOut(0);
  private final DutyCycleOut indexMotor_dutyCycleOut = new DutyCycleOut(0);
  private final PositionVoltage intakeMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage indexMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final DigitalInput intakeUpperSensor = new DigitalInput(IntakeConstants.kIntakeUpperSensorId);
  private final DigitalInput intakeLowerSensor = new DigitalInput(IntakeConstants.kIntakeLowerSensorId);
  private final DigitalInput intakeSensor2 = new DigitalInput(IntakeConstants.kIntakeLowerSensor2Id);
  private final DigitalInput intakeSensor3 = new DigitalInput(IntakeConstants.kIntakeUpperSensor3Id);


  public CommandIntakeIdle commandIntakeIdle = new CommandIntakeIdle(this);
  public CommandIntakeStart commandIntakeStart = new CommandIntakeStart(this);
  public CommandIntakeIntake commandIntakeIntake = new CommandIntakeIntake(this);
  public CommandIntakeProcess commandIntakeProcess = new CommandIntakeProcess(this);
  public CommandIntakeHold commandIntakeHold = new CommandIntakeHold(this);
  public CommandIntakeIndex commandIntakeIndex = new CommandIntakeIndex(this);

  public CommandIndexIdle commandIndexIdle = new CommandIndexIdle(this);
  public CommandIndexStart commandIndexStart = new CommandIndexStart(this);
  public CommandIndexIntake commandIndexIntake = new CommandIndexIntake(this);
  public CommandIndexProcess commandIndexProcess = new CommandIndexProcess(this);
  public CommandIndexHold commandIndexHold = new CommandIndexHold(this);
  public CommandIndexFire commandIndexFire = new CommandIndexFire(this);

  // private final double prematchDelay = 2.5;

  private double intakeMotorPos;
  private double indexMotorPos;
  private double intakeMotorSpeed;
  private double indexMotorSpeed;

  /** Creates a new intake. */
  public Intake() {
    isUpperNotePresent = false;
    isLowerNotePresent = false;

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeMotor.getConfigurator().apply(configs);
      status = indexMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    optimization_for_CAN();

    // USE NEXT LINE FOR TESTING
    PhysicsSim.getInstance().addTalonFX(intakeMotor, 0.001);
    PhysicsSim.getInstance().addTalonFX(indexMotor, 0.001);
  }

  public boolean lowerSensorDetectsNote() {
    return isLowerNotePresent || isNEWSENSORPresent || isNEWSENSOR2Present;
  }

  public boolean upperSensorDetectsNote() {
    return isUpperNotePresent;
  }

  public void setLowerSensorDetectsNote(boolean value) {
    isLowerNotePresent = value;
  }

  public void setUpperSensorDetectsNote(boolean value) {
    isUpperNotePresent = value;
  }

  public void intakeMotorToPosition(double rotations) {
    intakeMotor.setControl(intakeMotor_voltagePosition.withPosition(rotations));
  }

  public void indexMotorToPosition(double rotations) {
    indexMotor.setControl(indexMotor_voltagePosition.withPosition(rotations));
  }

  public double getIntakeMotorPosition() {
    return intakeMotorPos;
  }

  public double getIndexMotorPosition() {
    return indexMotorPos;
  }

  public void startIntakeMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIntakeStart
    intakeMotor.setControl(intakeMotor_dutyCycleOut.withOutput(-IntakeConstants.kIntakeMotorSpeed));
  }

  // public void reverseIntakeMotor() {
  //   intakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
  // }

  public void startIndexMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIndexStart
    indexMotor.setControl(indexMotor_dutyCycleOut.withOutput(-IntakeConstants.kIndexMotorSpeed));
  }

  public void startIntakeIndexerMotors(){
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIntakeStart & CommandOverrideIndexStart
    startIndexMotor();
    startIntakeMotor();
  }

  public void stopIntakeIndexerMotors(){
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIntakeStop & CommandOverrideIndexStop
    stopIntakeMotor();
    stopIndexMotor();
  }

  public void stopIntakeMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIntakeStop
    intakeMotor.setControl(new NeutralOut());
  }

  public void stopIndexMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIndexStop
    indexMotor.setControl(new NeutralOut());
  }

  public double getIntakeMotorSpeed() {
    return intakeMotorSpeed;
  }

  public double getIndexMotorSpeed() {
    return indexMotorSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!simulationDebugMode) {
      isUpperNotePresent = !intakeUpperSensor.get();
      isLowerNotePresent = !intakeLowerSensor.get();
      isNEWSENSORPresent = !intakeSensor2.get();
      isNEWSENSOR2Present = !intakeSensor3.get();
    }
  
    intakeMotorPos = intakeMotor.getPosition().getValueAsDouble();
    indexMotorPos = indexMotor.getPosition().getValueAsDouble();
    intakeMotorSpeed = intakeMotor.getDutyCycle().getValueAsDouble();
    indexMotorSpeed = indexMotor.getDutyCycle().getValueAsDouble();

    SmartDashboard.putBoolean("intakeUpperSensor state", isUpperNotePresent);
    SmartDashboard.putBoolean("intakeLowerSensor state", isLowerNotePresent);
    SmartDashboard.putBoolean("NEWSENSOR2 state", isNEWSENSORPresent);
    SmartDashboard.putBoolean("NEWSENSOR3 state", isNEWSENSOR2Present);
    SmartDashboard.putNumber("Intake Motor Temperature", intakeMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Index Motor Temperature", indexMotor.getDeviceTemp().getValueAsDouble());
  }
  
  // USE FOR TESTING ALSO
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  /*
   * ===============================
   * 
   *    PUBLIC COMPOSED COMMANDS
   * 
   * ===============================
   */

  /*
   * This is the public command that collects the note if allowed
   */
  public ConditionalCommand collectNote() {
    System.out.println("collectNote CONDITIONAL CHECK: " + IntakeConstants.currentIntakeState + " = " + IntakeConstants.intakeState.IDLE);
    return intakeNoteCollection()
      .onlyIf(
        () -> (IntakeConstants.currentIntakeState == IntakeConstants.intakeState.IDLE)
      );
  }

  /*
   * This is the public command that runs handoff if allowed
   */
  public ConditionalCommand passNoteToIndex() {
    System.out.println("passNoteToIndex CONDITIONAL CHECK: " + handoffAllowed());
    return intakeAndIndexHandoff()
      .onlyIf(
        () -> handoffAllowed()
      );
  }

  /*
   * This is the public command that runs the index side of note firing if allowed
   */
  public ConditionalCommand fireNote() {
    System.out.println("fireNote CONDITIONAL CHECK: " + IntakeConstants.currentIndexState + " = " + IntakeConstants.indexState.HOLD);
    return indexFireNote()
      .onlyIf(
        () -> (IntakeConstants.currentIndexState == IntakeConstants.indexState.HOLD)
      );
  }

  /*
   * =====================================
   * 
   *    PRIVATE COMPOSED COMMAND PIECES
   * 
   * =====================================
   */


  /*
   * This is a command chain for the intake side of handoff
   */
  private SequentialCommandGroup intakeNoteCollection() {
    return new SequentialCommandGroup(
      new PrintCommand("intakeNoteCollection step1"),
      commandIntakeStart,
      new PrintCommand("intakeNoteCollection step2"),
      commandIntakeIntake,
      new PrintCommand("intakeNoteCollection step3"),
      commandIntakeProcess,
      new PrintCommand("intakeNoteCollection step4"),
      commandIntakeHold,
      new PrintCommand("intakeNoteCollection step5")
    );
  }
  
  /*
  * This is a command chain that runs both sides if handoff at the same time
  */
  private SequentialCommandGroup intakeAndIndexHandoff() {
    return new SequentialCommandGroup(
      new PrintCommand("intakeAndIndexHandoff step1"),
      commandIndexStart,
      new PrintCommand("intakeAndIndexHandoff step2"),
      commandIntakeIndex,
      new PrintCommand("intakeAndIndexHandoff step3"),
      commandIndexIntake,
      new PrintCommand("intakeAndIndexHandoff step4"),
      commandIndexProcess,
      new PrintCommand("intakeAndIndexHandoff step5"),
      commandIntakeIdle,
      new PrintCommand("intakeAndIndexHandoff step6"),
      commandIndexHold,
      new PrintCommand("intakeAndIndexHandoff step7")
    );
  }

  /*
   * This determines if we're allowed to run handoff
   */
  private boolean handoffAllowed() {
    return IntakeConstants.currentIntakeState == IntakeConstants.intakeState.HOLD && SATConstants.state == SATConstants.Position.START;
  }

  /*
   * This is a command chain for the index side of note firing
   */
  private SequentialCommandGroup indexFireNote() {
    return new SequentialCommandGroup(
      new PrintCommand("indexFireNote step1"),
      commandIndexFire,
      new PrintCommand("indexFireNote step2"),
      commandIndexIdle,
      new PrintCommand("indexFireNote step3")
    );
  }

  public void optimization_for_CAN() {
    StatusSignal<Double> m_IntakeMotor_canbus1signal1 = intakeMotor.getPosition();
    StatusSignal<Double> m_IndexMotor_canbus1signal2 = indexMotor.getPosition();
    StatusSignal<Double> m_IntakeTemp_canbus1signal1 = intakeMotor.getDeviceTemp();
    StatusSignal<Double> m_IndexTemp_canbus1signal2 = indexMotor.getDeviceTemp();
    StatusSignal<Double> m_IntakeDutyCycle_canbus1signal1 = intakeMotor.getDutyCycle();
    StatusSignal<Double> m_IndexDutyCycle_canbus1signal2 = indexMotor.getDutyCycle();
    BaseStatusSignal.setUpdateFrequencyForAll(60, m_IntakeMotor_canbus1signal1, m_IndexMotor_canbus1signal2, m_IntakeDutyCycle_canbus1signal1, m_IndexDutyCycle_canbus1signal2);
    BaseStatusSignal.setUpdateFrequencyForAll(1, m_IntakeTemp_canbus1signal1, m_IndexTemp_canbus1signal2);
    ParentDevice.optimizeBusUtilizationForAll(intakeMotor, indexMotor);
  }
}
