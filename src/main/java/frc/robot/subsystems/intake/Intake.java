// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

public class Intake extends SubsystemBase {
  boolean intakeSensorState;
  
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);


  DigitalInput intakeSensor = new DigitalInput(0);


  public final TalonFX m_fx = new TalonFX(1, "rio");
  public final NeutralOut m_brake = new NeutralOut();
  private final double prematchDelay = 2.5;

  /** Creates a new intake. */
  public Intake() {
    
  }

  public boolean isIRBeamBreakBroken() {
    return !intakeSensor.get();
  }

  public void feedToShooter() {
    intakeMotor.set(IntakeConstants.intakeMotorSpeed);
  }

  public void outtakeNoteInIntake() {
    intakeMotor.set(-IntakeConstants.intakeMotorSpeed);
  }

  public void stopIntakeMotor() {
    intakeMotor.set(0.0);
  }


  /*The function below is a duplicate */
  public void Intake() {
    if (!isIRBeamBreakBroken()) {
      feedToShooter();
    } 
    else {
      stopIntakeMotor();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Note?", isIRBeamBreakBroken());
    //SmartDashboard.putNumber("Intake Motor Velocity", getVelocity());
  }
}
