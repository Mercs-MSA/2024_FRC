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
  private final TalonFX satShooter1Motor = new TalonFX(SATConstants.SAT_SHOOTER1_MOTOR_ID);
  private final TalonFX satShooter2Motor = new TalonFX(SATConstants.SAT_SHOOTER2_MOTOR_ID);

  private final Follower Base2_Follower = new Follower(SATConstants.SAT_BASE1_MOTOR_ID, true);
  private final Follower Shooter2_Follower = new Follower(SATConstants.SAT_SHOOTER1_MOTOR_ID, true);

  
  
  private final PositionVoltage satPivotMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage satBase1_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage satBase2_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
   
  XboxController controller = new XboxController(3);

   

  /**this was named by gaurav*/
  Servo SATTEamisDumb = new Servo(Constants.SATConstants.SAT_SERVO1_SERVO_ID);
  Servo SATServo2 = new Servo(Constants.SATConstants.SAT_SERVO2_SERVO_ID);
  Servo SATServo3 = new Servo(Constants.SATConstants.SAT_SERVO3_SERVO_ID);
  Servo SATServo4 = new Servo(Constants.SATConstants.SAT_SERVO4_SERVO_ID);

  boolean B_Button_Value;
  double Y_axis_Value;


  private final DigitalInput satObjectDectecter = new DigitalInput(Constants.SATConstants.SAT_OBJECTDETECTOR_SENSOR_ID);
 
    

  
  
    // Peak output of 8 volts

  public SAT() {
  /**this stuff happens ONCE, when the code enables, NOT WHEN THE ROBOT ENABLES */
  TalonFXConfiguration satBase1MotorConfigs = new TalonFXConfiguration();
  satBase1MotorConfigs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
  satBase1MotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
  // Peak output of 8 volts
  satBase1MotorConfigs.Voltage.PeakForwardVoltage = 8;
  satBase1MotorConfigs.Voltage.PeakReverseVoltage = -8;

  TalonFXConfiguration satBase2MotorConfigs = new TalonFXConfiguration();
  satBase2MotorConfigs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
  satBase2MotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
  // Peak output of 8 volts
  satBase2MotorConfigs.Voltage.PeakForwardVoltage = 8;
  satBase2MotorConfigs.Voltage.PeakReverseVoltage = -8;

  TalonFXConfiguration satPivotMotorConfigs = new TalonFXConfiguration();
  satPivotMotorConfigs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
  satPivotMotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
  // Peak output of 8 volts
  satPivotMotorConfigs.Voltage.PeakForwardVoltage = 8;
  satPivotMotorConfigs.Voltage.PeakReverseVoltage = -8;


  TalonFXConfiguration satShooter1MotorConfigs = new TalonFXConfiguration();
  satShooter1MotorConfigs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
  satShooter1MotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
  // Peak output of 8 volts
  satShooter1MotorConfigs.Voltage.PeakForwardVoltage = 8;
  satShooter1MotorConfigs.Voltage.PeakReverseVoltage = -8;

  TalonFXConfiguration satShooter2MotorConfigs = new TalonFXConfiguration();
  satShooter2MotorConfigs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
  satShooter2MotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
  // Peak output of 8 volts
  satShooter2MotorConfigs.Voltage.PeakForwardVoltage = 8;
  satShooter2MotorConfigs.Voltage.PeakReverseVoltage = -8;



//STATUS FOR BASE1
  StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = satBase1Motor.getConfigurator().apply(satShooter1MotorConfigs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

//STATUS FOR BASE2
    StatusCode status2 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = satBase2Motor.getConfigurator().apply(satBase2MotorConfigs);
      if (status2.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

//STATUS FOR pIVOT
    StatusCode status3 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = satPivotMotor.getConfigurator().apply(satPivotMotorConfigs);
      if (status3.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    //STATUS FOR SHOOTER1
    StatusCode status4 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status4 = satShooter1Motor.getConfigurator().apply(satShooter1MotorConfigs);
      if (status4.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status4.toString());
    }

    //STATUS FOR SHOOTER2
    StatusCode status5 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status5 = satShooter2Motor.getConfigurator().apply(satShooter2MotorConfigs);
      if (status5.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status5.toString());
    }
    
    
    


    /*PUT FOLLOW SYSTEMS HERE */
    satBase2Motor.setControl(Base2_Follower);
    satShooter2Motor.setControl(Shooter2_Follower);

  
    satBase1Motor.setPosition(0);
    satBase2Motor.setPosition(0);
    satPivotMotor.setPosition(0);
    
  }


 
  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run


   
  }  


    public void goToBasePodiumPosition(){
      satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.BASE_PODIUM_POS));
        satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.BASE_PODIUM_POS));
    }
    

  
    public void goBaseToSubPosition() {
        satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.BASE_SUB_POS));
        satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.BASE_SUB_POS));
      }
    

    public void goToBaseAmpPosition() {
     
        satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.BASE_AMP_POS));
        satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.BASE_AMP_POS));
        

    }

    public void goToBaseTrapPosition() {
  
        satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.BASE_TRAP_POS));
        satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.BASE_TRAP_POS));
    }
    
    public void goToBaseZeroPosition() {
     
        satBase1Motor.setControl(satBase1_voltagePosition.withPosition(0));
        satBase2Motor.setControl(satBase2_voltagePosition.withPosition(0));
        

    }

    public void shootNote() {
     
        satShooter1Motor.set(Constants.SATConstants.SHOOTER_SPEED);
        satShooter2Motor.set(Constants.SATConstants.SHOOTER_SPEED);

    }
}