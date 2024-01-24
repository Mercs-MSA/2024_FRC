// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SAT;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkFlexExternalEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SAT extends SubsystemBase {
  /** Creates a new SAT. */
   CANSparkFlex satShooter1Motor = new CANSparkFlex(SATConstants.SAT_SHOOTER1_MOTOR_ID, MotorType.kBrushless);
   CANSparkFlex satShooter2Motor = new CANSparkFlex(SATConstants.SAT_SHOOTER2_MOTOR_ID, MotorType.kBrushless);
   
   CANSparkFlex satPivotMotor = new CANSparkFlex(SATConstants.SAT_PIVOT_MOTOR_ID, MotorType.kBrushless);
   CANSparkFlex satBase1Motor = new CANSparkFlex(SATConstants.SAT_BASE1_MOTOR_ID, MotorType.kBrushless);

   CANSparkFlex satBase2Motor = new CANSparkFlex(SATConstants.SAT_BASE2_MOTOR_ID, MotorType.kBrushless);
   

   XboxController controller = new XboxController(3);

   SparkPIDController Base1_pidController = satBase1Motor.getPIDController();
   SparkPIDController Base2_pidController = satBase2Motor.getPIDController();
   SparkPIDController Pivot_pidController = satPivotMotor.getPIDController();
   SparkPIDController Shooter1_pidController = satShooter1Motor.getPIDController();
   SparkPIDController Shooter2_pidController = satShooter2Motor.getPIDController();

   /**this was named by gaurav*/
   Servo SATTEamisDumb = new Servo(Constants.SATConstants.SAT_SERVO1_SERVO_ID);
   Servo SATServo2 = new Servo(Constants.SATConstants.SAT_SERVO2_SERVO_ID);
   Servo SATServo3 = new Servo(Constants.SATConstants.SAT_SERVO3_SERVO_ID);
   Servo SATServo4 = new Servo(Constants.SATConstants.SAT_SERVO4_SERVO_ID);

   private final DigitalInput satObjectDectecter = new DigitalInput(Constants.SATConstants.SAT_OBJECTDETECTOR_SENSOR_ID);
   DutyCycleEncoder BaseThroughboreEncoder = new DutyCycleEncoder(Constants.SATConstants.BASE_THROUGHBORE_ENCODER);
    DutyCycleEncoder PivotThroughboreEncoder = new DutyCycleEncoder(Constants.SATConstants.PIVOT_THROUGHBORE_ENCODER);

  public SAT() {
  /**this stuff happens ONCE, when the code enables, NOT WHEN THE ROBOT ENABLES */
    satBase1Motor.restoreFactoryDefaults();
    satBase2Motor.restoreFactoryDefaults();
    satShooter1Motor.restoreFactoryDefaults();
    satShooter2Motor.restoreFactoryDefaults();
    satPivotMotor.restoreFactoryDefaults();


    satBase2Motor.follow(satBase1Motor, true);
    satShooter2Motor.follow(satShooter1Motor, true);

  /*JUST IN CASE THE ROBOT STARTS IN THE WRONG POSITION, THIS WILL FIX IT */
  final double translatedBasePos = get_sat_base_throughbore_position()*Constants.SATConstants.BASE_ENCODER_RATIO;
  final double translatedPivotPos = get_sat_pivot_throughbore_position()*Constants.SATConstants.PIVOT_ENCODER_RATIO;    
  /*TRANSLATED POS IS THE POSITION THAT IT ACTUALLY IS, WHAT THE THROUGHBORE KNOWS IT IS */

  satBase1Motor.getEncoder().setPosition(translatedBasePos);
  satPivotMotor.getEncoder().setPosition(translatedPivotPos);





    

    
  }


 
  @Override
  public void periodic() {


    // This method will be called once per scheduler run


    if (controller.getBButton() && (controller.getRawAxis(1) < -0.5)) {
     /**pODIUM*/
     Pivot_pidController.setReference(Constants.SATConstants.PIVOT_PODIUM_POS, CANSparkMax.ControlType.kPosition);
   

    }
    else if (controller.getBButton() && (controller.getRawAxis(1) > 0.5)) {
     /** SUBWOOFER */
     Pivot_pidController.setReference(Constants.SATConstants.PIVOT_AMP_POS, CANSparkMax.ControlType.kPosition);
   

    }
    else if (controller.getBButton() && (controller.getRawAxis(0) < -0.5)) {
     /** AMP */
     Pivot_pidController.setReference(Constants.SATConstants.PIVOT_SUB_POS, CANSparkMax.ControlType.kPosition);}
    
    else if (controller.getBButton() && (controller.getRawAxis(0) < -0.5)) {
     /** AMP */
     Pivot_pidController.setReference(Constants.SATConstants.PIVOT_SUB_POS, CANSparkMax.ControlType.kPosition);

    }
    if (controller.getBButton() && (controller.getRawAxis(0) > 0.5)) {
     /** TRAP */
     Pivot_pidController.setReference(40, CANSparkMax.ControlType.kPosition);
   

    }
    if (controller.getBButton() && (controller.getRawAxis(0) < -0.5)) {
     /** ZERO */
     Pivot_pidController.setReference(40, CANSparkMax.ControlType.kPosition);
   

    }

    
    
  }

  
    
     
  

    public double get_sat_base_throughbore_position() {
       return BaseThroughboreEncoder.getAbsolutePosition();
    }
     
    public double get_sat_base_motor_position(){
        
      return satBase1Motor.getEncoder().getPosition();

    }
    public double get_sat_pivot_throughbore_position() {
      return PivotThroughboreEncoder.getAbsolutePosition();
    }

    public double get_sat_pivot_motor_position(){
        return satPivotMotor.getEncoder().getPosition();

    }


    

    
      

   


}