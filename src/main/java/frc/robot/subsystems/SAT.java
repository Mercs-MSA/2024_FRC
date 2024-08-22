// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.sim.PhysicsSim;

public class SAT extends SubsystemBase {
  /** Creates a new SAT. */
  private final TalonFX satPivotMotor = new TalonFX(SATConstants.SAT_PIVOT_MOTOR_ID);
  
  private final TalonFX satShooterLeftMotor = new TalonFX(SATConstants.SAT_SHOOTER_LEFT_MOTOR_ID);
  private final TalonFX satShooterRightMotor = new TalonFX(SATConstants.SAT_SHOOTER_RIGHT_MOTOR_ID);
  private final Follower LeftFollower = new Follower(23, true);
  

  private final PositionVoltage satPivotMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);


  private double base1MotorPos, base2MotorPos, pivotMotorPos, shooterMotorBottomSpeed, Base1StartPosition, Base2StartPosition, PivotStartPosition;
  public double shooterMotorLeftSpeed;
  private double baseTargetPose, pivotTargetPose = 0.0;

  private TalonFXConfiguration satBase1MotorConfigs, satBase2MotorConfigs;

  public int works = 5;


  // private final DigitalInput satObjectDectecter = new DigitalInput(Constants.SATConstants.SAT_OBJECTDETECTOR_SENSOR_ID);

  // Peak output of 8 volts

  public SAT() {
    /**
     * this stuff happens ONCE, when the code enables, NOT WHEN THE ROBOT ENABLES
     */
   

    
    TalonFXConfiguration satPivotMotorConfigs = new TalonFXConfiguration();
    satPivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    satPivotMotorConfigs.Slot0.kP = 2; // An error of 0.5 rotations results in 1.2 volts output
    satPivotMotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    satPivotMotorConfigs.Voltage.PeakForwardVoltage = 16;
    satPivotMotorConfigs.Voltage.PeakReverseVoltage = -16;
  

    TalonFXConfiguration satShooteTopMotorConfigs = new TalonFXConfiguration();
    satShooteTopMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    satShooteTopMotorConfigs.Slot0.kP = 0.2; // An error of 0.5 rotations results in 1.2 volts output
    satShooteTopMotorConfigs.Slot0.kS = 0.05; // Add 0.05 V output to overcome static friction
    satShooteTopMotorConfigs.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    satShooteTopMotorConfigs.Slot0.kI = 0; // no output for integrated error
    satShooteTopMotorConfigs.Slot0.kD = 0; // no output for error derivative
    
    satShooteTopMotorConfigs.Voltage.PeakForwardVoltage = 15;
    satShooteTopMotorConfigs.Voltage.PeakReverseVoltage = -15;
    satShooteTopMotorConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    satShooteTopMotorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;  
    satShooteTopMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    satShooteTopMotorConfigs.CurrentLimits.StatorCurrentLimit = 35;

  TalonFXConfiguration satShooterBottomMotorConfigs = new TalonFXConfiguration();
    satShooterBottomMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    satShooterBottomMotorConfigs.Slot0.kP = 0.2; // An error of 0.5 rotations results in 1.2 volts output
    satShooterBottomMotorConfigs.Slot0.kS = 0.05; // Add 0.05 V output to overcome static friction
    satShooterBottomMotorConfigs.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    satShooterBottomMotorConfigs.Slot0.kI = 0; // no output for integrated error
    satShooterBottomMotorConfigs.Slot0.kD = 0; // no output for error derivative
    
    satShooterBottomMotorConfigs.Voltage.PeakForwardVoltage = 15;
    satShooterBottomMotorConfigs.Voltage.PeakReverseVoltage = -15;
    satShooterBottomMotorConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    satShooterBottomMotorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;  
    satShooterBottomMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    satShooterBottomMotorConfigs.CurrentLimits.StatorCurrentLimit = 35;
   

    // STATUS FOR PIVOT
    StatusCode status3 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status3 = satPivotMotor.getConfigurator().apply(satPivotMotorConfigs);
      if (status3.isOK())
        break;
    }
    if (!status3.isOK()) {
      System.out.println("Could not apply configs, error code: " + status3.toString());
    }

    // STATUS FOR SHOOTER1
    StatusCode status4 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status4 = satShooterLeftMotor.getConfigurator().apply(satShooteTopMotorConfigs);
      if (status4.isOK())
        break;
    }
    if (!status4.isOK()) {
      System.out.println("Could not apply configs, error code: " + status4.toString());
    }

    // STATUS FOR SHOOTER2
    StatusCode status5 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status5 = satShooterRightMotor.getConfigurator().apply(satShooterBottomMotorConfigs);
      if (status5.isOK())
        break;
    }
    if (!status5.isOK()) {
      System.out.println("Could not apply configs, error code: " + status5.toString());
    }

    /* PUT FOLLOW SYSTEMS HERE */
    
    PivotStartPosition = satPivotMotor.getPosition().getValueAsDouble();

    optimization_for_CAN();

    PhysicsSim.getInstance().addTalonFX(satPivotMotor, 0.001);
    PhysicsSim.getInstance().addTalonFX(satShooterLeftMotor, 0.001);
    PhysicsSim.getInstance().addTalonFX(satShooterRightMotor, 0.001);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    pivotMotorPos = satPivotMotor.getPosition().getValueAsDouble();
    shooterMotorLeftSpeed = satShooterLeftMotor.getVelocity().getValueAsDouble();
    shooterMotorBottomSpeed = satShooterRightMotor.getVelocity().getValueAsDouble();

    SmartDashboard.putNumber("base1MotorPos", base1MotorPos);
    SmartDashboard.putNumber("base2MotorPos", base2MotorPos);

    SmartDashboard.putNumber("base1PosRelativeToStart", base1MotorPos - Base1StartPosition);
    SmartDashboard.putNumber("base2PosRelativeToStart", base2MotorPos - Base2StartPosition);

  
    SmartDashboard.putNumber("pivotMotorPosRaw", pivotMotorPos);
    SmartDashboard.putNumber("shooter command", satShooterLeftMotor.getClosedLoopReference().getValueAsDouble());

    
    SmartDashboard.putNumber("Pivot Motor Temperature", satPivotMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Motor1 Temperature", satShooterLeftMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Motor2 Temperature", satShooterRightMotor.getDeviceTemp().getValueAsDouble());

    SmartDashboard.putNumber("shooterMotor1Speed", shooterMotorLeftSpeed);
    SmartDashboard.putNumber("shooterMotor2Speed", shooterMotorBottomSpeed);

    SmartDashboard.putNumber("shooter1MotorVoltage", satShooterLeftMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("shooter2MotorVoltage", satShooterRightMotor.getMotorVoltage().getValueAsDouble());

    SmartDashboard.putNumber("shooter1MotorVoltage", satShooterLeftMotor.getStatorCurrent().getValue());
    SmartDashboard.putNumber("shooter2MotorVoltage", satShooterRightMotor.getStatorCurrent().getValue());



  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
  
  // This is for test purposes only


  public boolean isWithinTol(double targetPose, double currentPose, double tolerance) {
    return (Math.abs(targetPose - currentPose) <= tolerance);
  }

  /**
   * Returns the Base 1 Motor's position, as cached by the SAT subsystem.
   */
  public double getBase1Pos() {
    return base1MotorPos;
  }

  public double getPivotPos() {
    return pivotMotorPos;
  }

  /**
   * Returns the Base 2 Motor's position, as cached by the SAT subsystem.
   */
  
  public double getShooterSpeed() {
    return shooterMotorLeftSpeed;
  }

  public void resetMotors(){
    satPivotMotor.setControl(new NeutralOut());
    satShooterLeftMotor.setControl(new NeutralOut());

  }

  public void neturalPivot(){
    satPivotMotor.setControl(new NeutralOut());
  }

  public void neturalBase(){
    satPivotMotor.setControl(new NeutralOut());
  }

  public void goToHomePos(){
    satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(Constants.SATConstants.START.pivot));

  }

  public StatusCode movePivot(double pos){
    works++;
    return satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(pos));
  }

  public void startShooter(double speed)
  {
    satShooterLeftMotor.setControl(new VoltageOut(speed));
    satShooterRightMotor.setControl(LeftFollower);
  }

  public void stopShooter() {
    satShooterLeftMotor.setControl(new NeutralOut());
    satShooterRightMotor.setControl(LeftFollower);
  }
  

  public void optimization_for_CAN() {
    StatusSignal<Double> m_PivotMotor_canbus1signal1 = satPivotMotor.getPosition();
   
    StatusSignal<Double> m_Shooter1Motor_canbus1signal4 = satShooterLeftMotor.getVelocity();
    StatusSignal<Double> m_Shooter2Motor_canbus1signal5 = satShooterRightMotor.getVelocity();
    StatusSignal<Double> m_PivotTemp_canbus1signal6 = satPivotMotor.getDeviceTemp();
   
    StatusSignal<Double> m_Shooter1Temp_canbus1signal9 = satShooterLeftMotor.getDeviceTemp();
    StatusSignal<Double> m_Shooter2Temp_canbus1signal10 = satShooterRightMotor.getDeviceTemp();
    
    StatusSignal<Double> m_Shooter1MotorClosed_canbus1signal15 = satShooterLeftMotor.getClosedLoopReference();   
    StatusSignal<Double> m_Shooter1Volt_canbus1signal16 = satShooterLeftMotor.getMotorVoltage();
    StatusSignal<Double> m_Shooter2Volt_canbus1signal17 = satShooterRightMotor.getMotorVoltage();
    BaseStatusSignal.setUpdateFrequencyForAll(60, m_PivotMotor_canbus1signal1, m_Shooter1Motor_canbus1signal4, m_Shooter2Motor_canbus1signal5, m_Shooter1MotorClosed_canbus1signal15, m_Shooter1Volt_canbus1signal16, m_Shooter2Volt_canbus1signal17);
    BaseStatusSignal.setUpdateFrequencyForAll(1, m_PivotTemp_canbus1signal6, m_Shooter1Temp_canbus1signal9, m_Shooter2Temp_canbus1signal10);
    ParentDevice.optimizeBusUtilizationForAll(satPivotMotor, satShooterLeftMotor, satShooterRightMotor);
  }
}