// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SATConstants;
import frc.robot.commands.IndexSubcommands.CommandIndexFire;
import frc.robot.commands.IndexSubcommands.CommandIndexHold;
import frc.robot.commands.IndexSubcommands.CommandIndexIdle;
import frc.robot.commands.IndexSubcommands.CommandIndexIntake;
import frc.robot.commands.IndexSubcommands.CommandIndexProcess;
import frc.robot.commands.IndexSubcommands.CommandIndexStart;
import frc.robot.commands.IntakeSubcommands.CommandIntakeHold;
import frc.robot.commands.IntakeSubcommands.CommandIntakeIdle;
import frc.robot.commands.IntakeSubcommands.CommandIntakeIndex;
import frc.robot.commands.IntakeSubcommands.CommandIntakeIntake;
import frc.robot.commands.IntakeSubcommands.CommandIntakeProcess;
import frc.robot.commands.IntakeSubcommands.CommandIntakeStart;
//  USE NEXT LINE FOR TESTING
import frc.robot.sim.PhysicsSim;

public class Intake extends SubsystemBase {
  // USE NEXT LINE FOR TESTING
  public boolean simulationDebugMode = Robot.isSimulation();

  private boolean isUpperNotePresent;
  private boolean isLowerNotePresent1;
  private boolean isLowerNotePresent2;
  private boolean isLowerNotePresent3;
  
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorId); //carpet
  private final TalonFX indexMotor = new TalonFX(IntakeConstants.kIndexMotorId); //sat (feeder)
  private final DutyCycleOut intakeMotor_dutyCycleOut = new DutyCycleOut(0);
  private final DutyCycleOut indexMotor_dutyCycleOut = new DutyCycleOut(0);
  private final PositionVoltage intakeMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage indexMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final DigitalInput intakeUpperSensor = new DigitalInput(IntakeConstants.kIntakeUpperSensorId);
  private final DigitalInput intakeSensor1 = new DigitalInput(IntakeConstants.kIntakeLowerSensor1Id);
  private final DigitalInput intakeSensor2 = new DigitalInput(IntakeConstants.kIntakeLowerSensor2Id);
  private final DigitalInput intakeSensor3 = new DigitalInput(IntakeConstants.kIntakeLowerSensor3Id);

  // Define the intakeUpperSensorCallback
/*   private BiConsumer<Boolean, Boolean> intakeUpperSensorCallback = (risingEdge, fallingEdge) -> {
    if (risingEdge) {
      SmartDashboard.putString("sensor debug", "upper - rising");
      setUpperSensorDetectsNote(true);
      // setLowerSensorDetectsNote(false);
    }
    if (fallingEdge) {
      SmartDashboard.putString("sensor debug", "upper - falling");
    }
  }; */

  // Define the intakeUpperSensorCallback
  // private BiConsumer<Boolean, Boolean> intakeLowerSensorCallback = (risingEdge, fallingEdge) -> {
  //   if (risingEdge) {
  //     SmartDashboard.putString("sensor debug", "lower - rising");
  //   }
  //   if (fallingEdge) {
  //     // setLowerSensorDetectsNote(true);
  //     SmartDashboard.putString("sensor debug", "lower - falling");
  //   }
  // };

  // AsynchronousInterrupt intakeSensor3Interrupt = new AsynchronousInterrupt(intakeSensor3, intakeLowerSensorCallback); 
  // AsynchronousInterrupt intakeSensor1Interrupt = new AsynchronousInterrupt(intakeSensor1, intakeLowerSensorCallback); 
  // AsynchronousInterrupt intakeSensor2Interrupt = new AsynchronousInterrupt(intakeSensor2, intakeLowerSensorCallback); 
  //AsynchronousInterrupt intakeUpperSensorInterrupt = new AsynchronousInterrupt(intakeUpperSensor, intakeUpperSensorCallback); 

  // private final double prematchDelay = 2.5;

  private double intakeMotorPos;
  private double indexMotorPos;
  private double intakeMotorSpeed;
  private double indexMotorSpeed;

  /** Creates a new intake. */
  public Intake() {
    SmartDashboard.putString("sensor debug", "init");

    //disableIntakeUpperSensorInterrupt();
    // disableIntakeLowerSensorInterrupt();
    // enableIntakeLowerSensorInterrupt();
    // enableIntakeUpperSensorInterrupt();

    isUpperNotePresent = false;
    isLowerNotePresent1 = false;
    isLowerNotePresent2 = false;
    isLowerNotePresent3 = false;

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
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
    return isLowerNotePresent1 || isLowerNotePresent2 || isLowerNotePresent3;
  }

  public boolean upperSensorDetectsNote() {
    return isUpperNotePresent;
  }

  public void setLowerSensorDetectsNote(boolean value) {
    isLowerNotePresent1 = value;
    isLowerNotePresent2 = value;
    isLowerNotePresent3 = value;
  }
  
  public void setUpperSensorDetectsNote(boolean value) {
    isUpperNotePresent = value;
  }

/*   public void enableIntakeUpperSensorInterrupt(){
    intakeUpperSensorInterrupt.setInterruptEdges(true, true);
    intakeUpperSensorInterrupt.enable();
  }

  public void disableIntakeUpperSensorInterrupt(){
    intakeUpperSensorInterrupt.disable();
  } */

  // public void enableIntakeLowerSensorInterrupt(){
  //   intakeSensor1Interrupt.setInterruptEdges(true, true);
  //   intakeSensor2Interrupt.setInterruptEdges(true, true);
  //   intakeSensor3Interrupt.setInterruptEdges(true, true);
  //   intakeSensor1Interrupt.enable();
  //   intakeSensor2Interrupt.enable();
  //   intakeSensor3Interrupt.enable();
  // }

  // public void disableIntakeLowerSensorInterrupt(){
  //   intakeSensor1Interrupt.disable();
  //   intakeSensor2Interrupt.disable();
  //   intakeSensor3Interrupt.disable();
  // }
  
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
      // We don't want to update the upper sensor in periodic because it's being
      // controlled by the AsynchronousInterrupt now.
       isUpperNotePresent = !intakeUpperSensor.get();
      isLowerNotePresent1 = !intakeSensor1.get();
      isLowerNotePresent2 = !intakeSensor2.get();
      isLowerNotePresent3 = !intakeSensor3.get();
    }
  
    intakeMotorPos = intakeMotor.getPosition().getValueAsDouble();
    indexMotorPos = indexMotor.getPosition().getValueAsDouble();
    intakeMotorSpeed = intakeMotor.getDutyCycle().getValueAsDouble();
    indexMotorSpeed = indexMotor.getDutyCycle().getValueAsDouble();

    SmartDashboard.putBoolean("Upper Sensor state", isUpperNotePresent);
    SmartDashboard.putBoolean("Lower Sensor1 state", isLowerNotePresent1);
    SmartDashboard.putBoolean("Lower Sensor2 state", isLowerNotePresent2);
    SmartDashboard.putBoolean("Lower Sensor3 state", isLowerNotePresent3);
    SmartDashboard.putNumber("Intake Motor Temperature", intakeMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Index Motor Temperature", indexMotor.getDeviceTemp().getValueAsDouble());
  }
  
  // USE FOR TESTING ALSO
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  /*
   * ====================================================================
   * 
   *    PUBLIC COMPOSED COMMANDS
   * 
   * ====================================================================
   */

  /*
   * This is the public command that collects the note if allowed.
   * This function really should not need to be changed.
   * The collection can only occur if the collectionAllowed() method returns true.
   * This gives us the ability to change the conditions for when collection is
   * allowed by changing the logic in that method, without affecting what
   * commands are needed to collect a note. There is a debug print statement that outputs
   * the current state in case things are not working.
   */
  public ConditionalCommand collectNote() {
    System.out.println("collectNote CONDITIONAL CHECK: " + IntakeConstants.currentIntakeState + " = " + IntakeConstants.intakeState.IDLE);
    return intakeNoteCollection()
      .onlyIf(
        () -> collectionAllowed()
      );
  }

  /*
   * This logic determines if we're allowed to run handoff.
   * The Intake must be holding a note and the SAT must be in the handoff position.
   */
  private boolean collectionAllowed() {
    return (
      IntakeConstants.currentIntakeState == IntakeConstants.intakeState.IDLE
    );
  }

  /*
   * This is the public command that runs handoff if allowed.
   * This function really should not need to be changed.
   * The handoff can only occur if the handoffAllowed() method returns true.
   * This gives us the ability to change the conditions for when a handoff is
   * allowed by changing the logic in that method, without affecting what
   * commands are needed to perform the handoff. There is a debug print statement
   * that outputs whether a handoff was allowed in case things are not working.
   */
  public ConditionalCommand passNoteToIndex() {
    System.out.println("passNoteToIndex CONDITIONAL CHECK: " + handoffAllowed());
    return intakeAndIndexHandoff()
      .onlyIf(
        () -> handoffAllowed()
      );
  }

  /*
   * This logic determines if we're allowed to run handoff.
   * The Intake must be holding a note and the SAT must be in the handoff position.
   */
  private boolean handoffAllowed() {
    return (true
      //IntakeConstants.currentIntakeState == IntakeConstants.intakeState.HOLD && 
      //SATConstants.state == SATConstants.Position.HANDOFF
    );
  }

  /*
   * This is the public command that runs the index side of note firing if allowed.
   * This function really should not need to be changed.
   * Firing can only occur if the shootingAllowed() method returns true.
   * This gives us the ability to change the conditions for when shooting is
   * allowed by changing the logic in that method, without affecting what
   * commands are needed to perform the shot. There is a debug print statement
   * that outputs the current state in case things are not working.
   */
  public ConditionalCommand fireNote() {
    System.out.println("fireNote CONDITIONAL CHECK: " + IntakeConstants.currentIndexState + " = " + IntakeConstants.indexState.HOLD);
    return indexFireNote()
      .onlyIf(
        () -> shootingAllowed()
      );
  }

  /*
   * This logic determines if we're allowed to shoot a note.
   * The Intake must be holding a note in the indexer.
   */
  private boolean shootingAllowed() {
    return (
      IntakeConstants.currentIndexState == IntakeConstants.indexState.HOLD
    );
  }


  /*
   * ==========================================================================
   * 
   *    PRIVATE COMPOSED COMMAND PIECES
   * 
   * ==========================================================================
   */


  /*
   * This is a command chain for the intake side of handoff. This can be edited to
   * change the sequence of what happens when a note is collected. Here's what it's
   * composed of:
   * 
   * 1. debug message
   * 2. start spinning intake rollers
   * 3. debug message
   * 4. watch for note to pass lower sensors
   * 5. debug message
   * 6. change intake motor movement to specific rotations and jog
   *    note inward a small amount to lift it off floor
   * 7. debug message
   * 8. stop intake rollers
   * 9. debug message
   * 
   * If this sequence fails in any way, use the debug messages to determine how far
   * into the sequence things worked. This helps you not incorrectly assume something
   * else was going wrong.
   */
  private SequentialCommandGroup intakeNoteCollection() {
    return new SequentialCommandGroup(
      new PrintCommand("intakeNoteCollection step1"),
      new CommandIntakeStart(this),
      new PrintCommand("intakeNoteCollection step2"),
      new CommandIntakeIntake(this),
      new PrintCommand("intakeNoteCollection step3"),
      //new CommandIntakeProcess(this),
      new PrintCommand("intakeNoteCollection step4"),
      //new CommandIntakeHold(this),
      new PrintCommand("intakeNoteCollection step5")
    );
  }
  
  /*
   * This is a command chain that runs both sides if handoff at the same time. 
   * This can be edited to change the sequence of what happens when a note is collected.
   * Here's what it's composed of:
   * 
   * 1. debug message
   * 2. start spinning indexer roller and enable interrupt
   * 3. debug message
   * 4. start spinning intake roller and wait for note to leave lower sensor
   * 5. debug message
   * 6. watch for note to pass upper sensor
   * 7. debug message
   * 8. change indexer motor movement to specific rotations and jog
   *    note inward a small amount to pull it all the way into SAT
   * 9. debug message
   * 10. stop intake rollers
   * 11. debug message
   * 12. stop indexer roller and disable interrupt
   * 13. debug message
   * 
   * If this sequence fails in any way, use the debug messages to determine how far
   * into the sequence things worked. This helps you not incorrectly assume something
   * else was going wrong.
   */
  private SequentialCommandGroup intakeAndIndexHandoff() {
    return new SequentialCommandGroup(
      new PrintCommand("intakeAndIndexHandoff step1"),
//      new CommandIndexStart(this),
      new PrintCommand("intakeAndIndexHandoff step2"),
//      new CommandIndexIntake(this),
      new PrintCommand("intakeAndIndexHandoff step3"),
//      new CommandIndexProcess(this),
      new WaitCommand(0.2),
      new PrintCommand("intakeAndIndexHandoff step4"),
      new CommandIntakeIdle(this),
      new PrintCommand("intakeAndIndexHandoff step5"),
      new CommandIndexHold(this),
      new PrintCommand("intakeAndIndexHandoff step6")
    );
  }

  /*
   * This is a command chain for the indexer side of note firing
   * This can be edited to change the sequence of what happens when a note is fired.
   * Here's what it's composed of:
   * 
   * 1. debug message
   * 2. start spinning indexer roller and wait for note to leave upper sensor
   * 3. debug message
   * 4. stop indexer roller
   * 5. debug message
   * 
   * If this sequence fails in any way, use the debug messages to determine how far
   * into the sequence things worked. This helps you not incorrectly assume something
   * else was going wrong.
   */
  private SequentialCommandGroup indexFireNote() {
    return new SequentialCommandGroup(
      new PrintCommand("indexFireNote step1"),
      new CommandIndexFire(this),
      new PrintCommand("indexFireNote step2"),
      new CommandIndexIdle(this),
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
    StatusSignal<Double> m_Shooter1Volt_canbus1signal3 = intakeMotor.getMotorVoltage();
    StatusSignal<Double> m_Shooter2Volt_canbus1signal4 = indexMotor.getMotorVoltage();
    BaseStatusSignal.setUpdateFrequencyForAll(60, m_IntakeMotor_canbus1signal1, m_IndexMotor_canbus1signal2, m_IntakeDutyCycle_canbus1signal1, m_IndexDutyCycle_canbus1signal2, m_Shooter1Volt_canbus1signal3, m_Shooter2Volt_canbus1signal4);
    BaseStatusSignal.setUpdateFrequencyForAll(1, m_IntakeTemp_canbus1signal1, m_IndexTemp_canbus1signal2);
    ParentDevice.optimizeBusUtilizationForAll(intakeMotor, indexMotor);
  }
}
