// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SAT;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkFlexExternalEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import javax.swing.plaf.TreeUI;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.CTREConfigs;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;


public class SAT extends SubsystemBase {
  /** Creates a new SAT. */
  private final TalonFX satPivotMotor = new TalonFX(SATConstants.SAT_PIVOT_MOTOR_ID);
  private final TalonFX satBase1Motor = new TalonFX(SATConstants.SAT_BASE1_MOTOR_ID);
  private final TalonFX satBase2Motor = new TalonFX(SATConstants.SAT_BASE2_MOTOR_ID);

  private final Follower Base2_Follower = new Follower(SATConstants.SAT_BASE1_MOTOR_ID, true);
  

  CANSparkFlex satShooter1Motor = new CANSparkFlex(SATConstants.SAT_SHOOTER1_MOTOR_ID, MotorType.kBrushless);
  CANSparkFlex satShooter2Motor = new CANSparkFlex(SATConstants.SAT_SHOOTER2_MOTOR_ID, MotorType.kBrushless);
   
  private final PositionVoltage satPivotMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage satBase1_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage satBase2_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
   
  XboxController controller = new XboxController(3);

   
  SparkPIDController Shooter1_pidController = satShooter1Motor.getPIDController();
  SparkPIDController Shooter2_pidController = satShooter2Motor.getPIDController();
  /**this was named by gaurav*/
  Servo SATTEamisDumb = new Servo(Constants.SATConstants.SAT_SERVO1_SERVO_ID);
  Servo SATServo2 = new Servo(Constants.SATConstants.SAT_SERVO2_SERVO_ID);
  Servo SATServo3 = new Servo(Constants.SATConstants.SAT_SERVO3_SERVO_ID);
  Servo SATServo4 = new Servo(Constants.SATConstants.SAT_SERVO4_SERVO_ID);

  boolean B_Button_Value;
  double Y_axis_Value;


  private final DigitalInput satObjectDectecter = new DigitalInput(Constants.SATConstants.SAT_OBJECTDETECTOR_SENSOR_ID);
 
    

  TalonFXConfiguration satBase1MotorConfigs = new TalonFXConfiguration();

  

  private static final double kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
  private static final double kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
       // Peak output of 8 volts
  private static final double PeakForwardVoltage = 8;
  private static final double PeakReverseVoltage = -8;
    // Peak output of 8 volts

  public SAT() {
  /**this stuff happens ONCE, when the code enables, NOT WHEN THE ROBOT ENABLES */

    
    satShooter1Motor.restoreFactoryDefaults();
    satShooter2Motor.restoreFactoryDefaults();
    satBase1Motor.getConfigurator().apply(satBase1MotorConfigs);
    satBase2Motor.getConfigurator();
    satPivotMotor.getConfigurator();


    /*PUT FOLLOW SYSTEMS HERE */
    satBase2Motor.setControl(Base2_Follower);
    satShooter2Motor.follow(satShooter1Motor, true);

  

    
  }


 
  @Override
  public void periodic() {
    B_Button_Value = controller.getBButton();
    Y_axis_Value = controller.getRawAxis(1);

    // This method will be called once per scheduler run


    if (B_Button_Value && (Y_axis_Value < -0.5)) {
     /**pODIUM*/
     
      satBase1Motor.setControl(satBase1_voltagePosition.withPosition(50));
      satBase2Motor.setControl(satBase2_voltagePosition.withPosition(50));

    }
    if (B_Button_Value && (Y_axis_Value > 0.5)) {
     /** SUBWOOFER */
     
      satBase1Motor.setControl(satBase1_voltagePosition.withPosition(150));
      satBase2Motor.setControl(satBase2_voltagePosition.withPosition(150));

    }
    if (B_Button_Value && (controller.getRawAxis(0) < -0.5)) {
    /** AMP */
      satBase1Motor.setControl(satBase1_voltagePosition.withPosition(150));
      satBase2Motor.setControl(satBase2_voltagePosition.withPosition(150));
      satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(150));
     
}
    
    if (controller.getBButton() && (controller.getRawAxis(0) > 0.5)) {
     /** TRAP */
    
      satBase1Motor.setControl(satBase1_voltagePosition.withPosition(150));
      satBase2Motor.setControl(satBase2_voltagePosition.withPosition(150));
      satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(150));

    }
    if (controller.getBButton() && (controller.getRawAxis(0) < -0.5)) {
     /** ZERO */
     
      satBase1Motor.setControl(satBase1_voltagePosition.withPosition(0));
      satBase2Motor.setControl(satBase2_voltagePosition.withPosition(0));
      satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(0));

    }    
  }  

  
    public Command goToPodiumPosition() {
      return this.runOnce(() -> {
        satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.BASE_PODIUM_POS));
        satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.BASE_PODIUM_POS));
      });
    }

    public Command goToSubPosition() {
      return this.runOnce(() -> {
        satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.BASE_SUB_POS));
        satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.BASE_SUB_POS));
      });
    }

    public Command goToAmpPosition() {
      return this.runOnce(() -> {
        satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.BASE_AMP_POS));
        satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.BASE_AMP_POS));
        satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(Constants.SATConstants.PIVOT_AMP_POS));
      });
    }

    public Command goToTrapPosition() {
      return this.runOnce(() -> {
        satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.BASE_TRAP_POS));
        satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.BASE_TRAP_POS));
        satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(Constants.SATConstants.PIVOT_TRAP_POS));
      });
    }
    
    public Command goToZeroPosition() {
      return this.runOnce(() -> {
        satBase1Motor.setControl(satBase1_voltagePosition.withPosition(0));
        satBase2Motor.setControl(satBase2_voltagePosition.withPosition(0));
        satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(0));
      });
    }
}