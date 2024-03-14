// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;

public class Index extends SubsystemBase {
  public boolean simulationDebugMode = Robot.isSimulation();

  private boolean isUpperNotePresent;

  
  private final TalonFX indexMotor = new TalonFX(IntakeConstants.kIndexMotorId);
  private final PositionVoltage indexMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final VelocityVoltage indexMotor_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  private final DigitalInput intakeUpperSensor = new DigitalInput(IntakeConstants.kIntakeUpperSensorId);

  private double indexMotorPos;
  private double indexMotorSpeed;

  BiConsumer<Boolean, Boolean> callback = (risingEdge, fallingEdge) -> {
    setUpperSensorDetectsNote(true);
  };

  private AsynchronousInterrupt asynchronousInterrupt = new AsynchronousInterrupt(intakeUpperSensor, callback);

  /** Creates a new intake. */
  public Index() {
    SmartDashboard.putString("sensor debug", "init");

    isUpperNotePresent = false;


    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.Slot0.kP = 25.0; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.4; // A change of 1 rotation per second results in 0.1 volts output

    configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 16;
    configs.Voltage.PeakReverseVoltage = -16;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 40;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = indexMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    optimization_for_CAN();
    asynchronousInterrupt.disable();

    // USE NEXT LINE FOR TESTING
    PhysicsSim.getInstance().addTalonFX(indexMotor, 0.001);
  }


  public boolean upperSensorDetectsNote() {
    return isUpperNotePresent;
  }

  public void setUpperSensorDetectsNote(boolean value) {
    isUpperNotePresent = value;
  }

  public void indexMotorToPosition(double rotations) {
    indexMotor.setControl(indexMotor_voltagePosition.withPosition(rotations));
  }

  public double getIndexMotorPosition() {
    return indexMotorPos;
  }

  public void enableAsynchronousInterrupt(){
    asynchronousInterrupt.enable();
  }

  public void disableAsynchronousInterrupt(){
    asynchronousInterrupt.disable();
  }

  
  public void resetMotors(){
    stopIndexMotor();
  }

  public void startIndexMotor() {
    indexMotor.setControl(indexMotor_voltageVelocity.withVelocity(Constants.IntakeConstants.kIndexMotorSpeed));
  }

  public void reverseIndexMotor() {
    indexMotor.setControl(indexMotor_voltageVelocity.withVelocity(Constants.IntakeConstants.kSlowIndexMotorSpeed));
  }

  public void stopIndexMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIndexStop
    indexMotor.setControl(new NeutralOut());
  }

  public double getIndexMotorSpeed() {
    return indexMotorSpeed;
  }


  @Override
  public void periodic() {
  
    indexMotorPos = indexMotor.getPosition().getValueAsDouble();
    indexMotorSpeed = indexMotor.getDutyCycle().getValueAsDouble();

    SmartDashboard.putBoolean("Upper Sensor state", isUpperNotePresent);
    SmartDashboard.putNumber("index motor speed", indexMotorSpeed);
    SmartDashboard.putNumber("Index Motor Temperature", indexMotor.getDeviceTemp().getValueAsDouble());
  }
  
  // USE FOR TESTING ALSO
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  public void optimization_for_CAN() {
    StatusSignal<Double> m_IndexMotor_canbus1signal2 = indexMotor.getPosition();
    StatusSignal<Double> m_IndexTemp_canbus1signal2 = indexMotor.getDeviceTemp();
    StatusSignal<Double> m_IndexDutyCycle_canbus1signal2 = indexMotor.getDutyCycle();
    StatusSignal<Double> m_Shooter2Volt_canbus1signal4 = indexMotor.getMotorVoltage();
    BaseStatusSignal.setUpdateFrequencyForAll(60, m_IndexMotor_canbus1signal2, m_IndexDutyCycle_canbus1signal2, m_Shooter2Volt_canbus1signal4);
    BaseStatusSignal.setUpdateFrequencyForAll(1, m_IndexTemp_canbus1signal2);
    ParentDevice.optimizeBusUtilizationForAll(indexMotor);
  }
}
