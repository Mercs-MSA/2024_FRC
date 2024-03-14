// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SATConstants;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
  /** Creates a new SAT. */
  private final TalonFX satShooter1Motor = new TalonFX(SATConstants.SAT_SHOOTER1_MOTOR_ID);
  private final TalonFX satShooter2Motor = new TalonFX(SATConstants.SAT_SHOOTER2_MOTOR_ID);

  private final Follower Shooter2_Follower = new Follower(SATConstants.SAT_SHOOTER1_MOTOR_ID, true);

  private final VelocityVoltage satShooter1_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  private double shooterMotor1Speed, shooterMotor2Speed;

  
  public Shooter() {
 
    TalonFXConfiguration satShooter1MotorConfigs = new TalonFXConfiguration();
    satShooter1MotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    satShooter1MotorConfigs.Slot0.kP = 0.2; // An error of 0.5 rotations results in 1.2 volts output
    satShooter1MotorConfigs.Slot0.kS = 0.05; // Add 0.05 V output to overcome static friction
    satShooter1MotorConfigs.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    satShooter1MotorConfigs.Slot0.kI = 0; // no output for integrated error
    satShooter1MotorConfigs.Slot0.kD = 0; // no output for error derivative
    
    satShooter1MotorConfigs.Voltage.PeakForwardVoltage = 15;
    satShooter1MotorConfigs.Voltage.PeakReverseVoltage = -15;
    satShooter1MotorConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    satShooter1MotorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;  
    satShooter1MotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    satShooter1MotorConfigs.CurrentLimits.StatorCurrentLimit = 35;



    // STATUS FOR SHOOTER1
    StatusCode status4 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
        status4 = satShooter1Motor.getConfigurator().apply(satShooter1MotorConfigs);
        if (status4.isOK())
        break;
    }
    if (!status4.isOK()) {
        System.out.println("Could not apply configs, error code: " + status4.toString());
    }

    // STATUS FOR SHOOTER2
    StatusCode status5 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
        status5 = satShooter2Motor.getConfigurator().apply(satShooter1MotorConfigs);
        if (status5.isOK())
        break;
    }
    if (!status5.isOK()) {
        System.out.println("Could not apply configs, error code: " + status5.toString());
    }

    /* PUT FOLLOW SYSTEMS HERE */
    satShooter2Motor.setControl(Shooter2_Follower);
    optimization_for_CAN();
    PhysicsSim.getInstance().addTalonFX(satShooter1Motor, 0.001);
    PhysicsSim.getInstance().addTalonFX(satShooter2Motor, 0.001);
    }

    @Override
    public void periodic() {

        shooterMotor1Speed = satShooter1Motor.getVelocity().getValueAsDouble();
        shooterMotor2Speed = satShooter2Motor.getVelocity().getValueAsDouble();

        SmartDashboard.putNumber("shooter command", satShooter1Motor.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("Shooter1 Motor Temperature", satShooter1Motor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Shooter2 Motor Temperature", satShooter2Motor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("shooter1 Motor Speed", shooterMotor1Speed);
        SmartDashboard.putNumber("shooter2 Motor Speed", shooterMotor2Speed);
        SmartDashboard.putNumber("shooter1 Motor Voltage", satShooter1Motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("shooter2 Motor Voltage", satShooter2Motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("shooter1 Motor current", satShooter1Motor.getStatorCurrent().getValue());
        SmartDashboard.putNumber("shooter2 Motor current", satShooter2Motor.getStatorCurrent().getValue());

    }

    @Override
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    public void shootNote(double speed){
        satShooter1Motor.setControl(satShooter1_voltageVelocity.withVelocity(speed)); 
    }

    public void stopShooter(){
        satShooter1Motor.setControl(new NeutralOut());
    }

    public void stopShooterBrake(){
        satShooter1Motor.setControl(satShooter1_voltageVelocity.withVelocity(0));
    }

    public double getShooterSpeed() {
        return shooterMotor1Speed;
    }

    public void resetMotors(){
        satShooter1Motor.setControl(new NeutralOut());
    }

    public void optimization_for_CAN() {
        StatusSignal<Double> m_Shooter1Motor_canbus1signal4 = satShooter1Motor.getVelocity();
        StatusSignal<Double> m_Shooter2Motor_canbus1signal5 = satShooter2Motor.getVelocity();
        StatusSignal<Double> m_Shooter1Temp_canbus1signal9 = satShooter1Motor.getDeviceTemp();
        StatusSignal<Double> m_Shooter2Temp_canbus1signal10 = satShooter2Motor.getDeviceTemp();
        StatusSignal<Double> m_Shooter1MotorClosed_canbus1signal15 = satShooter1Motor.getClosedLoopReference();   
        StatusSignal<Double> m_Shooter1Volt_canbus1signal16 = satShooter1Motor.getMotorVoltage();
        StatusSignal<Double> m_Shooter2Volt_canbus1signal17 = satShooter2Motor.getMotorVoltage();
        BaseStatusSignal.setUpdateFrequencyForAll(60, m_Shooter1Motor_canbus1signal4, m_Shooter2Motor_canbus1signal5, m_Shooter1MotorClosed_canbus1signal15, m_Shooter1Volt_canbus1signal16, m_Shooter2Volt_canbus1signal17);
        BaseStatusSignal.setUpdateFrequencyForAll(1, m_Shooter1Temp_canbus1signal9, m_Shooter2Temp_canbus1signal10);
        ParentDevice.optimizeBusUtilizationForAll(satShooter1Motor, satShooter2Motor);
    }
}