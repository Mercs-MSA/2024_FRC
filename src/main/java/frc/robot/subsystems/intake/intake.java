// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Controller;
import edu.wpi.first.wpilibj2.command.Command;
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

//  USE NEXT LINE FOR TESTING
//  import frc.robot.sim.PhysicsSim;

/* Work In Progress
 * ON HOLD use/connect robotState to our code
 */

/* Q&A:
 * 1 Have you figured out what motors you want to use?
 *   Using vortexes for motors for now
 * 
 * 2 How does the mechanism work?
 *   System drops to floor from bottom of robot and takes in notes
 *   Request for buttons to control 
 *   Holding button down keeps the system down and releasing button moves system back up
 * 
 * 3 Have you decided if there will be special sensors?
 *   Using the REV 2m distance senor
 * 
 * 4 How does the note get to the SAT system?
 *   An index that uses rollers to transport it into the SAT, not part of the intake
 * 
 * 5 How does the machine realize it has an onion ring?
 *   Sensors inside the index, but it's not important to the intake system itself.
 * 
 * 6 What button do you want to be assigned to operate the intake system?
 *   TODO:
 * 
 * 
 */

public class Intake extends SubsystemBase {
  boolean intakeSensorState;
  public final double speedRollerInward = 1.0;
  public final double speedRollerOff = 0.0;
  public final double speedRollerOutward = -1.0;
  public final double positionArmDown = 0.001;
  public final double positionArmUp = 2.1;

  CANSparkFlex intakeRollerMotor = new CANSparkFlex(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  DigitalInput intakeSensor = new DigitalInput(0);
  public final TalonFX m_fx = new TalonFX(1, "rio");
  public final PositionVoltage m_voltagePosition = new PositionVoltage(positionArmUp, 0, true, 0, 0, false, false, false);
  public final NeutralOut m_brake = new NeutralOut();

  /** Creates a new intake. */
  public Intake() {
    // This method will be called once (at the beginning)

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // USE NEXT LINE FOR TESTING
    // PhysicsSim.getInstance().addTalonFX(m_fx, 0.001);
  }

  // USE FOR TESTING ALSO
  // @Override
  // public void simulationPeriodic() {
  //   PhysicsSim.getInstance().run();
  // }

  @Override
  public void periodic() {
    m_fx.getPosition().refresh();

    // constantly check sensor for note
    if (intakeSensor.get()) {
      // this is the state where it doesn't detect the orage (for some reason)
      intakeSensorState = false;
    }
    else {
      // when the note is present it changes states
      intakeSensorState = true;
    }

    SmartDashboard.putBoolean("IsButtonPressed", Controller.getAButton());
    SmartDashboard.putBoolean("Detecting Note", intakeSensorState);
    SmartDashboard.putNumber("MotorPosition", m_fx.getPosition().getValue());
  }

  // Activate the intake wheels
  public Command rollerSpeed(double speed) {
    return this.runOnce(()-> {
      intakeRollerMotor.set(speed);
    });
  }
}