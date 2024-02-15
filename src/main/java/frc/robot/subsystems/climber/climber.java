// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants;

//  USE NEXT LINE FOR TESTING
import frc.robot.sim.PhysicsSim;

public class climber extends SubsystemBase {
  private TalonFX tubeMotorLeft = new TalonFX(Constants.climberConstants.tubeMotor_Left_ID);
  private TalonFX tubeMotorRight = new TalonFX(Constants.climberConstants.tubeMotor_Right_ID);

  private final PositionVoltage tubeMotorRight_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage tubeMotorLeft_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
    
  private double rightMotorPosition;
  private double leftMotorPosition;

  public climberStates my_climber_state = climberStates.START;
  // private double targetPose = leftMotorPosition;
        
  /* TODO:
   * - setDirection() changes the variable depending on direction
   * - figure joysticks
   * - don't need Transit state?!
   */

  public climber() {
    TalonFXConfiguration climberRightMotorConfigs = new TalonFXConfiguration();
    climberRightMotorConfigs.Slot0.kP = 5.0; // An error of 0.5 rotations results in 1.2 volts output
    climberRightMotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    climberRightMotorConfigs.Voltage.PeakForwardVoltage = 2;
    climberRightMotorConfigs.Voltage.PeakReverseVoltage = -2;

    TalonFXConfiguration climberLeftMotorConfigs = new TalonFXConfiguration();
    climberLeftMotorConfigs.Slot0.kP = 5.0; // An error of 0.5 rotations results in 1.2 volts output
    climberLeftMotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    climberLeftMotorConfigs.Voltage.PeakForwardVoltage = 2;
    climberLeftMotorConfigs.Voltage.PeakReverseVoltage = -2;

    tubeMotorLeft.getConfigurator().apply(climberRightMotorConfigs);
    tubeMotorRight.getConfigurator().apply(climberLeftMotorConfigs);

    // USE NEXT LINE FOR TESTING
    PhysicsSim.getInstance().addTalonFX(tubeMotorLeft, 0.001);
    PhysicsSim.getInstance().addTalonFX(tubeMotorRight, 0.001);
  }
  
  public double outputRightData() {
    return rightMotorPosition;
  }

  public double outputLeftData() {
    return leftMotorPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightMotorPosition = tubeMotorRight.getPosition().getValueAsDouble();
    leftMotorPosition = tubeMotorLeft.getPosition().getValueAsDouble();

    SmartDashboard.putNumber("Motor Right State", outputRightData());
    SmartDashboard.putNumber("Motor Left State", outputLeftData());
  }
  
  // USE FOR TESTING ALSO
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  /**
   * This method prepares the robot for climbing in the middle of the chain
   */
  private void climbRightUp() {
    if (rightMotorPosition < Constants.climberConstants.RIGHT_TOP_POSITION) {
      tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(rightMotorPosition + Constants.climberConstants.climber_Increment));
    }
  }

  private void climbRightDown() {
    if (rightMotorPosition > Constants.climberConstants.RIGHT_BOTTOM_POSITION) {
      tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(rightMotorPosition - Constants.climberConstants.climber_Increment));
    }
  }

  private void climbLeftUp() {
    if (leftMotorPosition > Constants.climberConstants.LEFT_TOP_POSITION) {
      tubeMotorLeft.setControl(tubeMotorLeft_voltagePosition.withPosition(leftMotorPosition - Constants.climberConstants.climber_Increment));
    }
  }

  private void climbLeftDown() {
    if (leftMotorPosition < Constants.climberConstants.LEFT_BOTTOM_POSITION) {
      tubeMotorLeft.setControl(tubeMotorLeft_voltagePosition.withPosition(leftMotorPosition + Constants.climberConstants.climber_Increment));
    }
  }

  private void climbUp() {
    tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(Constants.climberConstants.RIGHT_TOP_POSITION));
    tubeMotorLeft.setControl(tubeMotorLeft_voltagePosition.withPosition(Constants.climberConstants.LEFT_TOP_POSITION));
  }

  private void climbDown() {
    tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(Constants.climberConstants.RIGHT_BOTTOM_POSITION));
    tubeMotorLeft.setControl(tubeMotorLeft_voltagePosition.withPosition(Constants.climberConstants.LEFT_BOTTOM_POSITION));
  }
 
  private void climbMidRight() {
    if (rightMotorPosition < Constants.climberConstants.RIGHT_TOP_POSITION) {
      tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(Constants.climberConstants.RIGHT_MID_POSITION));
    }
  }

  private void climbMidLeft() {
      tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(Constants.climberConstants.LEFT_MID_POSITION));
  }

  private void climbStop() {
    tubeMotorRight.setControl(new NeutralOut());
    tubeMotorLeft.setControl(new NeutralOut());
  }

  // public void leftGoToPosition(double joystick){
  //   targetPose = leftMotorPosition + (0.05*joystick);
  //   if (targetPose > 0 && targetPose < 14){
  //     tubeMotorLeft.setControl(tubeMotorLeft_voltagePosition.withPosition(targetPose));
  //   }
  // }

  // public void leftGoToPosition(double joystick){
  //   targetPose = leftMotorPosition + (0.05*joystick);
  //   if (targetPose > 0 && targetPose < 14){
  //     tubeMotorLeft.setControl(tubeMotorLeft_voltagePosition.withPosition(targetPose));
  //   }
  // }

  // Block of all commands
  public Command climbUpCommand() {
    return new RunCommand(() -> climbUp(), this);
  }

  public Command climbDownCommand() {
    return new RunCommand(() -> climbDown(), this);
  }

  public Command climbMidRightCommand() {
    return new RunCommand(() -> climbMidRight(), this);
  }

  public Command climbMidLeftCommand() {
    return new RunCommand(() -> climbMidLeft(), this);
  }

  public Command climbUpRightCommand() {
    return new RunCommand(() -> climbRightUp(), this);
  }

  public Command climbDownRightCommand() {
    return new RunCommand(() -> climbRightDown(), this);
  }

  public Command climbUpLeftCommand() {
    return new RunCommand(() -> climbLeftUp(), this);
  }

  public Command climbDownLeftCommand() {
    return new RunCommand(() -> climbLeftDown(), this);
  }

  public Command climbMotorStop() {
    return new RunCommand(() -> climbStop(), this);
  }
 
  // State Enumeration
  public enum climberStates {
    START, MOVING_TO_CLIMB, IN_CLIMBING_POSITION, COMPLETED_CLIMB, ERROR;
  }
}