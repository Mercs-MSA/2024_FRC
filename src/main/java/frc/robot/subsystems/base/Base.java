// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.base;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.sim.PhysicsSim;

public class Base extends SubsystemBase {
  private final TalonFX satBase1Motor = new TalonFX(SATConstants.SAT_BASE1_MOTOR_ID);
  private final TalonFX satBase2Motor = new TalonFX(SATConstants.SAT_BASE2_MOTOR_ID);

  private final Follower Base2_Follower = new Follower(SATConstants.SAT_BASE1_MOTOR_ID, true);

  private final PositionVoltage satBase1_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

  private double base1MotorPos, base2MotorPos, base1StartPosition;
  private double baseTargetPose = 0.0;


  public Base() {
    TalonFXConfiguration satBase1MotorConfigs = new TalonFXConfiguration();
    satBase1MotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    satBase1MotorConfigs.Slot0.kP = 2.0; // An error of 0.5 rotations results in 1.2 volts output
    satBase1MotorConfigs.Slot0.kD = 0; // A change of 1 rotation per second results in 0 volts output
    satBase1MotorConfigs.Slot0.kG = 0.0;
    satBase1MotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Peak output of 8 volts
    satBase1MotorConfigs.Voltage.PeakForwardVoltage = 14;
    satBase1MotorConfigs.Voltage.PeakReverseVoltage = -14;
    satBase1MotorConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    satBase1MotorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    StatusCode status1 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status1 = satBase1Motor.getConfigurator().apply(satBase1MotorConfigs);
      if (status1.isOK())
        break;
    }
    if (!status1.isOK()) {
      System.out.println("Could not apply configs, error code: " + status1.toString());
    }

    // STATUS FOR BASE2
    StatusCode status2 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status2 = satBase2Motor.getConfigurator().apply(satBase1MotorConfigs);
      if (status2.isOK())
        break;
    }
    if (!status2.isOK()) {
      System.out.println("Could not apply configs, error code: " + status2.toString());
    }

    satBase2Motor.setControl(Base2_Follower);

    optimization_for_CAN();

    PhysicsSim.getInstance().addTalonFX(satBase1Motor, 0.001);
    PhysicsSim.getInstance().addTalonFX(satBase2Motor, 0.001);

  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    base1MotorPos = satBase1Motor.getPosition().getValueAsDouble();
    base2MotorPos = satBase2Motor.getPosition().getValueAsDouble();


    SmartDashboard.putNumber("base1MotorPos", base1MotorPos);
    SmartDashboard.putNumber("base2MotorPos", base2MotorPos);

    SmartDashboard.putNumber("base1 command", satBase1Motor.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putNumber("base2 command", satBase2Motor.getClosedLoopReference().getValueAsDouble());

    SmartDashboard.putNumber("base1MotorVoltage", satBase1Motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("base2MotorVoltage", satBase2Motor.getMotorVoltage().getValueAsDouble());

    SmartDashboard.putNumber("Base1 Motor Temperature", satBase1Motor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Base2 Motor Temperature", satBase2Motor.getDeviceTemp().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
  
  // This is for test purposes only
  public void baseGoToPositionIncrement(double increment) {
    baseTargetPose = baseTargetPose + (increment);
    satBase1Motor.setControl(satBase1_voltagePosition.withPosition(base1StartPosition + baseTargetPose));
    // satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Base2StartPosition + baseTargetPose));
  }

  //
  public void moveBaseMotors(double base1Pos){
    satBase1Motor.setControl(satBase1_voltagePosition.withPosition(base1Pos)); 
  }

  public boolean isWithinTol(double targetPose, double currentPose, double tolerance) {
    return (Math.abs(targetPose - currentPose) <= tolerance);
  }

  public double getBase1Pos() {
    return base1MotorPos;
  }


  public double getBase2Pos() {
    return base2MotorPos;
  }


  public void resetMotors(){
    satBase1Motor.setControl(new NeutralOut());
    satBase2Motor.setControl(new NeutralOut());

  }

  public void goToHomePos(){
    satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.START.motor1_base));
  }

  public void optimization_for_CAN() {
    StatusSignal<Double> m_Base1Motor_canbus1signal2 = satBase1Motor.getPosition();
    StatusSignal<Double> m_Base2Motor_canbus1signal3 = satBase2Motor.getPosition();
    StatusSignal<Double> m_Base1Temp_canbus1signal7 = satBase1Motor.getDeviceTemp();
    StatusSignal<Double> m_Base2Temp_canbus1signal8 = satBase2Motor.getDeviceTemp();
    StatusSignal<Double> m_Base1MotorVolt_canbus1signal11 = satBase1Motor.getMotorVoltage();
    StatusSignal<Double> m_Base2MotorVolt_canbus1signal12 = satBase2Motor.getMotorVoltage();
    StatusSignal<Double> m_Base1MotorClosed_canbus1signal13 = satBase1Motor.getClosedLoopReference();
    StatusSignal<Double> m_Base2MotorClosed_canbus1signal14 = satBase2Motor.getClosedLoopReference();
    BaseStatusSignal.setUpdateFrequencyForAll(60, m_Base1Motor_canbus1signal2, m_Base2Motor_canbus1signal3, m_Base1MotorVolt_canbus1signal11, m_Base2MotorVolt_canbus1signal12, m_Base1MotorClosed_canbus1signal13, m_Base2MotorClosed_canbus1signal14);
    BaseStatusSignal.setUpdateFrequencyForAll(1, m_Base1Temp_canbus1signal7, m_Base2Temp_canbus1signal8);
    ParentDevice.optimizeBusUtilizationForAll(satBase1Motor, satBase2Motor);
  }
}