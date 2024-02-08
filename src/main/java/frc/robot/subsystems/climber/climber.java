// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.climber;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

//  USE NEXT LINE FOR TESTING
import frc.robot.sim.PhysicsSim;

public class climber extends SubsystemBase {
  /** Creates a new climber. */
    //state variables!
      //CANSparkMax tubeMotorLeft = new CANSparkMax(Constants.Climber.tubeMotor_Left_ID, MotorType.kBrushless);
      //CANSparkMax tubeMotorRight = new CANSparkMax(Constants.Climber.tubeMotor_Right_ID, MotorType.kBrushless);
      TalonFX tubeMotorLeft = new TalonFX(Constants.climberConstants.tubeMotor_Left_ID);
      TalonFX tubeMotorRight = new TalonFX(Constants.climberConstants.tubeMotor_Right_ID);

      private final PositionVoltage tubeMotorRight_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
      private final PositionVoltage tubeMotorLeft_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
      //int leftState = 0;
      //int rightState = 0;
     
      double rightMotorPosition;
      double leftMotorPosition;
      climberStates my_climber_state = climberStates.START;
          
      /* TODO:
        - setDirection() changes the variable depending on direction
        - figure joysticks
        - don't need Transit state?!
       */

  public climber() {

    TalonFXConfiguration climberRightMotorConfigs = new TalonFXConfiguration();
    climberRightMotorConfigs.Slot0.kP = 5.0; // An error of 0.5 rotations results in 1.2 volts output
    climberRightMotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    climberRightMotorConfigs.Voltage.PeakForwardVoltage = 14;
    climberRightMotorConfigs.Voltage.PeakReverseVoltage = -14;

    TalonFXConfiguration climberLeftMotorConfigs = new TalonFXConfiguration();
    climberLeftMotorConfigs.Slot0.kP = 5.0; // An error of 0.5 rotations results in 1.2 volts output
    climberLeftMotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    climberLeftMotorConfigs.Voltage.PeakForwardVoltage = 14;
    climberLeftMotorConfigs.Voltage.PeakReverseVoltage = -14;


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
    //manualControl();
    //buttonControl();

    rightMotorPosition = tubeMotorRight.getPosition().getValue();
    leftMotorPosition = tubeMotorLeft.getPosition().getValue();
    // This method will be called once per scheduler run
  // This moves the tube in tube up and down
  }
  
  // USE FOR TESTING ALSO
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  // private void manualControl() {
  //   if (Left_Joystick < 0.5) {
  //     climbDownLeftCommand();
  //   }
 
  //   else if (Left_Joystick < -0.5) {
  //     climbUpRightCommand();
  //   }
   
  //   else if (Right_Joystick < -0.5) {
  //     climbUpLeftCommand();
  //   }
 
  //   else if (Right_Joystick < 0.5) {
  //     climbDownRightCommand();
  //   }
 

  // } 

    /**
   * This method prepares the robot for climbing in the middle of the chain
   */

  private void climbRightUp() {
    if (rightMotorPosition < Constants.climberConstants.RIGHT_TOP_POSITION) {
      tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(rightMotorPosition + Constants.climberConstants.climber_Increment));
      //realPosition += Constants.climberConstants.climber_Increment;
      //tubeMotorRight.set(0.3);
      SmartDashboard.putString("power input for robot", "0.3");
    }
}
  private void climbRightDown() {
    if (rightMotorPosition > Constants.climberConstants.RIGHT_BOTTOM_POSITION) {
      tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(rightMotorPosition - Constants.climberConstants.climber_Increment));
      //tubeMotorRight.set(-0.3);
      SmartDashboard.putString("power input for robot", "-0.3");
    }
  }
  private void climbLeftUp() {
    if (leftMotorPosition > Constants.climberConstants.LEFT_TOP_POSITION) {
      tubeMotorLeft.setControl(tubeMotorLeft_voltagePosition.withPosition(leftMotorPosition - Constants.climberConstants.climber_Increment));
      //tubeMotorLeft.set(0.3);
      SmartDashboard.putString("power input for robot", "0.3");
    }
  }

  private void climbLeftDown() {
    if (leftMotorPosition < Constants.climberConstants.LEFT_BOTTOM_POSITION) {
      tubeMotorLeft.setControl(tubeMotorLeft_voltagePosition.withPosition(leftMotorPosition + Constants.climberConstants.climber_Increment));
      //tubeMotorLeft.set(-0.3);
      SmartDashboard.putString("power input for robot", "-0.3");
    }
}



// Block of all commands
public Command climbUpRightCommand() {
  return new RunCommand(() -> climbRightUp(), this);
}

public Command climbDownRightCommand() {
  return new RunCommand(() -> climbRightDown(), this);
}

public Command climbDownLeftCommand() {
  return new RunCommand(() -> climbLeftDown(), this);
}

public Command climbUpLeftCommand() {
  return new RunCommand(() -> climbLeftUp(), this);
}

 
// State Enumeration
  public enum climberStates {
    START, MOVING_TO_CLIMB, IN_CLIMBING_POSITION, COMPLETED_CLIMB, ERROR;
  }
}