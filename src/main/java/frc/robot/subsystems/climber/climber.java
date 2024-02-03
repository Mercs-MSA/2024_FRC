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
import com.ctre.phoenix6.controls.PositionVoltage;


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
      //XboxController controller = new XboxController(0);
     
      //double Left_Joystick;
      //double Right_Joystick;
      double rightMotorPosition;
      double leftMotorPosition;
      climberStates my_climber_state = climberStates.START;
          
      /* TODO:
        - setDirection() changes the variable depending on direction
        - figure joysticks
        - don't need Transit state?!
       */

  public climber() {
    tubeMotorLeft.getConfigurator();
    tubeMotorRight.getConfigurator();
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
    //Left_Joystick = controller.getLeftY();
    //Right_Joystick = controller.getRightY();

    rightMotorPosition = tubeMotorRight.getPosition().getValue();
    leftMotorPosition = tubeMotorLeft.getPosition().getValue();
    // This method will be called once per scheduler run
  // This moves the tube in tube up and down
  }

/*   private void buttonControl() {
    if (controller.getYButton()) {
      buttonClimbTest();
    }
    else if (controller.getBButton()) {
      buttonClimbTest2();
    }
    else if (controller.getAButton()){
      buttonBottomClimb();
    }
  }

  private void manualControl() {
    if (Left_Joystick < 0.5) {
      climbLeftDown();
    }
 
    else if (Left_Joystick < -0.5) {
      climbRightUp();
    }
   
    else if (Right_Joystick < -0.5) {
      climbLeftUp();
    }
 
    else if (Right_Joystick < 0.5) {
      climbLeftDown();
    }
 
    else {
      climbStop();
    }
  } */

    /**
   * This method prepares the robot for climbing in the middle of the chain
   */
  private void buttonBottomClimb() {
    tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(Constants.climberConstants.bottom_climber_position));
    tubeMotorLeft.setControl(tubeMotorRight_voltagePosition.withPosition(Constants.climberConstants.bottom_climber_position));
  }

  private void buttonClimbTest() {
    tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(Constants.climberConstants.test_climber_position));
    tubeMotorLeft.setControl(tubeMotorRight_voltagePosition.withPosition(Constants.climberConstants.test_climber_position));
  }

  private void buttonClimbTest2() {
    tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(Constants.climberConstants.test_climber_position));
    tubeMotorLeft.setControl(tubeMotorRight_voltagePosition.withPosition(Constants.climberConstants.test2_climber_position));
  }

  private void climbRightUp() {
    if (rightMotorPosition > -1000) {
      tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(rightMotorPosition + Constants.climberConstants.climber_Increment));
      //realPosition += Constants.climberConstants.climber_Increment;
      //tubeMotorRight.set(0.3);
      SmartDashboard.putString("power input for robot", "0.3");
    }
}
  private void climbRightDown() {
    if (rightMotorPosition < 0) {
      tubeMotorRight.setControl(tubeMotorRight_voltagePosition.withPosition(rightMotorPosition - Constants.climberConstants.climber_Increment));
      //tubeMotorRight.set(-0.3);
      SmartDashboard.putString("power input for robot", "-0.3");
    }
  }
  private void climbLeftUp() {
    if (leftMotorPosition > -1000) {
      tubeMotorLeft.setControl(tubeMotorLeft_voltagePosition.withPosition(leftMotorPosition + Constants.climberConstants.climber_Increment));
      //tubeMotorLeft.set(0.3);
      SmartDashboard.putString("power input for robot", "0.3");
    }
  }

  private void climbLeftDown() {
    if (leftMotorPosition < 0) {
      tubeMotorLeft.setControl(tubeMotorLeft_voltagePosition.withPosition(leftMotorPosition - Constants.climberConstants.climber_Increment));
      //tubeMotorLeft.set(-0.3);
      SmartDashboard.putString("power input for robot", "-0.3");
    }
}

  private void climbStop() {
    tubeMotorLeft.set(0);
    tubeMotorRight.set(0);
    SmartDashboard.putString("power input for robot", "0");
  }

// Block of all commands
public Command climbUpRightCommand() {
  return this.runOnce(() -> climbRightUp());
}

public Command climbDownRightCommand() {
  return this.runOnce(() -> climbRightDown());
}

public Command climbDownLeftCommand() {
  return this.runOnce(() -> climbLeftDown());
}

public Command climbUpLeftCommand() {
  return this.runOnce(() -> climbLeftUp());
}

public Command bottomButtonCommand() {
  return this.runOnce(() -> bottomButtonCommand());
}

public Command buttonTestCommand() {
  return this.runOnce(() -> buttonClimbTest());
}

public Command buttonTest2Command() {
  return this.runOnce(() -> buttonClimbTest2());
}
 
// State Enumeration
  public enum climberStates {
    START, MOVING_TO_CLIMB, IN_CLIMBING_POSITION, COMPLETED_CLIMB, ERROR;
  }
}