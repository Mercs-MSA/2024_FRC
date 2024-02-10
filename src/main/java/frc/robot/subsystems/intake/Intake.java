// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

//  USE NEXT LINE FOR TESTING
import frc.robot.sim.PhysicsSim;

// TODO: WRITE SPECIAL COMMANDS FOR JUST THE INDEX MOTOR
/* 
 * New command class X
 * new function in subsystem for just index X
 * named command in robot container 
 * New command in robot container
 */

// TODO: Add second sensor
// TODO: Stateful sensor logic
// TODO: Make detectNote() a trigger

public class Intake extends SubsystemBase {
  private boolean isNotePresent;
  
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorId); //carpet
  private final TalonFX indexMotor = new TalonFX(IntakeConstants.kIndexMotorId); //sat (feeder)
  private final DigitalInput intakeUpperSensor = new DigitalInput(IntakeConstants.kIntakeUpperSensorId);
  private final DigitalInput intakeLowerSensor = new DigitalInput(IntakeConstants.kIntakeLowerSensorId);

  // private final double prematchDelay = 2.5;

  /** Creates a new intake. */
  public Intake() {
    isNotePresent = false;

    TalonFXConfiguration configs = new TalonFXConfiguration();
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

    // USE NEXT LINE FOR TESTING
    PhysicsSim.getInstance().addTalonFX(intakeMotor, 0.001);
    PhysicsSim.getInstance().addTalonFX(indexMotor, 0.001);
  }

  public boolean detectNote() {
    return isNotePresent;
  }

  public void startIntakeMotor() {
    intakeMotor.set(-IntakeConstants.kIntakeMotorSpeed);
  }

  public void reverseIntakeMotor() {
    intakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
  }

  public void stopIntakeMotor() {
    intakeMotor.set(0.0);
  }

  public void startIndexMotor() {
    indexMotor.set(-IntakeConstants.kIndexMotorSpeed);
    SmartDashboard.putString("iS intake running?", "yes");
  }

  public void startIntakeIndexerMotors(){
    startIndexMotor();
    startIntakeMotor();
  }

  public void stopIntakeIndexerMotors(){
    stopIntakeMotor();
    stopIndexMotor();
  }

  public void stopIndexMotor() {
    indexMotor.set(0);
  }

  public double getIntakeMotorSpeed() {
    return intakeMotor.get();
  }
  public double getIndexMotorSpeed() {
    return indexMotor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // isNotePresent = !intakeUpperSensor.get();
    // isNotePresent = !intakeLowerSensor.get();
  }
  
  // USE FOR TESTING ALSO
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
