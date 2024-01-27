// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class climber extends SubsystemBase {
  /** Creates a new climber. */

    //state variables! 


      //CANSparkMax tubeMotorLeft = new CANSparkMax(Constants.Climber.tubeMotor_Left_ID, MotorType.kBrushless);
      //CANSparkMax tubeMotorRight = new CANSparkMax(Constants.Climber.tubeMotor_Right_ID, MotorType.kBrushless);

      TalonFX tubeMotorLeft = new TalonFX(Constants.Climber.tubeMotor_Left_ID);
      TalonFX tubeMotorRight = new TalonFX(Constants.Climber.tubeMotor_Right_ID);


      int leftState = 0;
      int rightState = 0; 
      XboxController controller = new XboxController(0);
      //1 - down; -1 - up
      int direction = 0;

      double Left_Joystick;
      double Right_Joystick;
      

      
      /* TODO:
        - setDirection() changes the variable depending on direction
        - figure joysticks
        - don't need Transit state?!
       */

      //  0 = starting state
      //  1 = Moving to prepare to climb
      //  2 = in climbing position
      //  3 = attempting to climb
      //  4 = completed climbing
      //  5 = error state

  public climber() {
  }


  @Override
  public void periodic() {
    Left_Joystick = controller.getLeftY();
    Right_Joystick = controller.getRightY();
    // This method will be called once per scheduler run
  // This moves the tube in tube up and down

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

  }

  private void climbRightUp() {
    tubeMotorRight.set(0.3);
    SmartDashboard.putString("power input for robot", "0.3");
}
  private void climbRightDown() {
    tubeMotorRight.set(-0.3);
    SmartDashboard.putString("power input for robot", "-0.3");
  }
  private void climbLeftUp() {
    tubeMotorLeft.set(0.3);
    SmartDashboard.putString("power input for robot", "0.3");
  }

  private void climbLeftDown() {
    tubeMotorLeft.set(-0.3);
    SmartDashboard.putString("power input for robot", "-0.3");
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


// State Enumeration
  public enum climberStates {
    START, MOVING_TO_CLIMB, IN_CLIMBING_POSITION, COMPLETED_CLIMB, ERROR;
  }

}