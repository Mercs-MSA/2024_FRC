// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
  private boolean isNotePresent;
  
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorId);
  private final DigitalInput intakeSensor = new DigitalInput(IntakeConstants.kIntakeSensorId);

  // private final double prematchDelay = 2.5;

  /** Creates a new intake. */
  public Intake() {
    isNotePresent = false;
  }
   // Make this a trigger
  public boolean detectNote() {
    return isNotePresent;
  }

  public void feedToShooter() {
    intakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
  }

  public void outtakeNoteInIntake() {
    intakeMotor.set(-IntakeConstants.kIntakeMotorSpeed);
  }

  public void stopIntakeMotor() {
    intakeMotor.set(0.0);
  }

  public void intakeAction() {
    // if there is no note
    if (!detectNote()) {
      // Take in the note
      feedToShooter();
    } 
    else {
      // Otherwise, stop the motors
      stopIntakeMotor();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isNotePresent = !intakeSensor.get();
    SmartDashboard.putBoolean("Intake Note?", detectNote());
    //SmartDashboard.putNumber("Intake Motor Velocity", getVelocity());
  }
}
