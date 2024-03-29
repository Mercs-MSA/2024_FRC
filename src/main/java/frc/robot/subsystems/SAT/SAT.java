// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SAT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.sim.PhysicsSim;

public class SAT extends SubsystemBase {
  /** Creates a new SAT. */
  private final TalonFX satPivotMotor = new TalonFX(SATConstants.SAT_PIVOT_MOTOR_ID);
  private final TalonFX satBase1Motor = new TalonFX(SATConstants.SAT_BASE1_MOTOR_ID);
  private final TalonFX satBase2Motor = new TalonFX(SATConstants.SAT_BASE2_MOTOR_ID);
  private final TalonFX satShooter1Motor = new TalonFX(SATConstants.SAT_SHOOTER1_MOTOR_ID);
  private final TalonFX satShooter2Motor = new TalonFX(SATConstants.SAT_SHOOTER2_MOTOR_ID);

  private final Follower Shooter2_Follower = new Follower(SATConstants.SAT_SHOOTER1_MOTOR_ID, true);
  private final Follower Base2_Follower = new Follower(SATConstants.SAT_BASE1_MOTOR_ID, true);

  private final VelocityVoltage satShooter1_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage satPivotMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage satBase1_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage satBase2_voltagePosition = satBase1_voltagePosition.clone();

  private double base1MotorPos, base2MotorPos, pivotMotorPos, shooterMotor1Speed, shooterMotor2Speed, Base1StartPosition, Base2StartPosition, PivotStartPosition;
  private double baseTargetPose, pivotTargetPose = 0.0;

  private TalonFXConfiguration satBase1MotorConfigs, satBase2MotorConfigs;

  // private final DigitalInput satObjectDectecter = new DigitalInput(Constants.SATConstants.SAT_OBJECTDETECTOR_SENSOR_ID);

  // Peak output of 8 volts

  public SAT() {
    /**
     * this stuff happens ONCE, when the code enables, NOT WHEN THE ROBOT ENABLES
     */
    satBase1MotorConfigs = new TalonFXConfiguration();
    satBase1MotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    satBase1MotorConfigs.Slot0.kP = 2.0; // An error of 0.5 rotations results in 1.2 volts output
    satBase1MotorConfigs.Slot0.kD = 0; // A change of 1 rotation per second results in 0 volts output
    satBase1MotorConfigs.Slot0.kG = 0.0;
    satBase1MotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Peak output of 8 volts
    satBase1MotorConfigs.Voltage.PeakForwardVoltage = 12;
    satBase1MotorConfigs.Voltage.PeakReverseVoltage = -12;
    satBase1MotorConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    satBase1MotorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    // satBase2MotorConfigs = new TalonFXConfiguration();
    // satBase2MotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // satBase2MotorConfigs.Slot0.kP = 1.0; // An error of 0.5 rotations results in 1.2 volts output
    // satBase2MotorConfigs.Slot0.kD = 0; // A change of 1 rotation per second results in 0 volts output
    // satBase2MotorConfigs.Slot0.kG = 0.0;
    // satBase2MotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // // Peak output of 8 volts
    // satBase2MotorConfigs.Voltage.PeakForwardVoltage = 14;
    // satBase2MotorConfigs.Voltage.PeakReverseVoltage = -14;
    // satBase2MotorConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    // satBase2MotorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;


    TalonFXConfiguration satPivotMotorConfigs = new TalonFXConfiguration();
    satPivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    satPivotMotorConfigs.Slot0.kP = 2; // An error of 0.5 rotations results in 1.2 volts output
    satPivotMotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    satPivotMotorConfigs.Voltage.PeakForwardVoltage = 16;
    satPivotMotorConfigs.Voltage.PeakReverseVoltage = -16;

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

    // STATUS FOR BASE1
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
    satBase2Motor.setControl(Base2_Follower);

    Base1StartPosition = satBase1Motor.getPosition().getValueAsDouble();
    Base2StartPosition = satBase2Motor.getPosition().getValueAsDouble();
    PivotStartPosition = satPivotMotor.getPosition().getValueAsDouble();

    optimization_for_CAN();

    PhysicsSim.getInstance().addTalonFX(satBase1Motor, 0.001);
    PhysicsSim.getInstance().addTalonFX(satBase2Motor, 0.001);
    PhysicsSim.getInstance().addTalonFX(satPivotMotor, 0.001);
    PhysicsSim.getInstance().addTalonFX(satShooter1Motor, 0.001);
    PhysicsSim.getInstance().addTalonFX(satShooter2Motor, 0.001);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    base1MotorPos = satBase1Motor.getPosition().getValueAsDouble();
    base2MotorPos = satBase2Motor.getPosition().getValueAsDouble();
    pivotMotorPos = satPivotMotor.getPosition().getValueAsDouble();
    shooterMotor1Speed = satShooter1Motor.getVelocity().getValueAsDouble();
    shooterMotor2Speed = satShooter2Motor.getVelocity().getValueAsDouble();

    SmartDashboard.putNumber("base1MotorPos", base1MotorPos);
    SmartDashboard.putNumber("base2MotorPos", base2MotorPos);

    SmartDashboard.putNumber("base1PosRelativeToStart", base1MotorPos - Base1StartPosition);
    SmartDashboard.putNumber("base2PosRelativeToStart", base2MotorPos - Base2StartPosition);

    SmartDashboard.putNumber("base1 command", satBase1Motor.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putNumber("base2 command", satBase2Motor.getClosedLoopReference().getValueAsDouble());

    SmartDashboard.putNumber("base1MotorVoltage", satBase1Motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("base2MotorVoltage", satBase2Motor.getMotorVoltage().getValueAsDouble());

    SmartDashboard.putNumber("pivotMotorPos", pivotMotorPos);
    SmartDashboard.putNumber("shooter command", satShooter1Motor.getClosedLoopReference().getValueAsDouble());

    SmartDashboard.putNumber("Base1 Motor Temperature", satBase1Motor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Base2 Motor Temperature", satBase2Motor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Motor Temperature", satPivotMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Motor1 Temperature", satShooter1Motor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Motor2 Temperature", satShooter2Motor.getDeviceTemp().getValueAsDouble());

    SmartDashboard.putNumber("shooterMotor1Speed", shooterMotor1Speed);
    SmartDashboard.putNumber("shooterMotor2Speed", shooterMotor2Speed);

    SmartDashboard.putNumber("shooter1MotorVoltage", satShooter1Motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("shooter2MotorVoltage", satShooter2Motor.getMotorVoltage().getValueAsDouble());

    SmartDashboard.putNumber("shooter1MotorVoltage", satShooter1Motor.getStatorCurrent().getValue());
    SmartDashboard.putNumber("shooter2MotorVoltage", satShooter2Motor.getStatorCurrent().getValue());

  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
  
  // This is for test purposes only
  public void baseGoToPositionIncrement(double increment) {
    baseTargetPose = baseTargetPose + (increment);
    satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Base1StartPosition + baseTargetPose));
    // satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Base2StartPosition + baseTargetPose));
  }

  // This is for test purposes only
  public void pivotGoToPositionIncrement(double increment) {
    // pivotTargetPose = pivotTargetPose + (increment);
    satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(this.pivotMotorPos + increment*6));
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

  public void movePivotMotor(double pos){
    satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(pos));
  }

  // public void moveBaseMotors(double base1Pos, double base2pos){
  //   satBase1Motor.setControl(satBase1_voltagePosition.withPosition(base1Pos)); 
  //   satBase2Motor.setControl(satBase2_voltagePosition.withPosition(base2pos));
  // }

  public void moveBaseMotors(double base1Pos){
    satBase1Motor.setControl(satBase1_voltagePosition.withPosition(base1Pos)); 
  }

  public boolean isWithinTol(double targetPose, double currentPose, double tolerance) {
    return (Math.abs(targetPose - currentPose) <= tolerance);
  }

  /**
   * Returns the Base 1 Motor's position, as cached by the SAT subsystem.
   */
  public double getBase1Pos() {
    return base1MotorPos;
  }

  /**
   * Returns the Base 2 Motor's position, as cached by the SAT subsystem.
   */
  public double getBase2Pos() {
    return base2MotorPos;
  }

  /**
   * Returns the Pivot Motor's position, as cached by the SAT subsystem.
   */
  public double getPivotPos() {
    return pivotMotorPos;
  }

  /**
   * Returns the Shooter Motor's position, as cached by the SAT subsystem.
   */
  public double getShooterSpeed() {
    return shooterMotor1Speed;
  }

  public void resetMotors(){
    satBase1Motor.setControl(new NeutralOut());
    satPivotMotor.setControl(new NeutralOut());
    satShooter1Motor.setControl(new NeutralOut());

  }

  public void neturalPivot(){
    satPivotMotor.setControl(new NeutralOut());
  }

  public void neturalBase(){
    satPivotMotor.setControl(new NeutralOut());
  }

  public void goToHomePos(){
    satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(Constants.SATConstants.START.pivot));
    satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.START.motor1_base));

  }

  public void optimization_for_CAN() {
    StatusSignal<Double> m_PivotMotor_canbus1signal1 = satPivotMotor.getPosition();
    StatusSignal<Double> m_Base1Motor_canbus1signal2 = satBase1Motor.getPosition();
    StatusSignal<Double> m_Base2Motor_canbus1signal3 = satBase2Motor.getPosition();
    StatusSignal<Double> m_Shooter1Motor_canbus1signal4 = satShooter1Motor.getVelocity();
    StatusSignal<Double> m_Shooter2Motor_canbus1signal5 = satShooter2Motor.getVelocity();
    StatusSignal<Double> m_PivotTemp_canbus1signal6 = satPivotMotor.getDeviceTemp();
    StatusSignal<Double> m_Base1Temp_canbus1signal7 = satBase1Motor.getDeviceTemp();
    StatusSignal<Double> m_Base2Temp_canbus1signal8 = satBase2Motor.getDeviceTemp();
    StatusSignal<Double> m_Shooter1Temp_canbus1signal9 = satShooter1Motor.getDeviceTemp();
    StatusSignal<Double> m_Shooter2Temp_canbus1signal10 = satShooter2Motor.getDeviceTemp();
    StatusSignal<Double> m_Base1MotorVolt_canbus1signal11 = satBase1Motor.getMotorVoltage();
    StatusSignal<Double> m_Base2MotorVolt_canbus1signal12 = satBase2Motor.getMotorVoltage();
    StatusSignal<Double> m_Base1MotorClosed_canbus1signal13 = satBase1Motor.getClosedLoopReference();
    StatusSignal<Double> m_Base2MotorClosed_canbus1signal14 = satBase2Motor.getClosedLoopReference();
    StatusSignal<Double> m_Shooter1MotorClosed_canbus1signal15 = satShooter1Motor.getClosedLoopReference();   
    StatusSignal<Double> m_Shooter1Volt_canbus1signal16 = satShooter1Motor.getMotorVoltage();
    StatusSignal<Double> m_Shooter2Volt_canbus1signal17 = satShooter2Motor.getMotorVoltage();
    BaseStatusSignal.setUpdateFrequencyForAll(60, m_PivotMotor_canbus1signal1, m_Base1Motor_canbus1signal2, m_Base2Motor_canbus1signal3, m_Shooter1Motor_canbus1signal4, m_Shooter2Motor_canbus1signal5, m_Base1MotorVolt_canbus1signal11, m_Base2MotorVolt_canbus1signal12, m_Base1MotorClosed_canbus1signal13, m_Base2MotorClosed_canbus1signal14, m_Shooter1MotorClosed_canbus1signal15, m_Shooter1Volt_canbus1signal16, m_Shooter2Volt_canbus1signal17);
    BaseStatusSignal.setUpdateFrequencyForAll(1, m_PivotTemp_canbus1signal6, m_Base1Temp_canbus1signal7, m_Base2Temp_canbus1signal8, m_Shooter1Temp_canbus1signal9, m_Shooter2Temp_canbus1signal10);
    ParentDevice.optimizeBusUtilizationForAll(satPivotMotor, satBase1Motor, satBase2Motor, satShooter1Motor, satShooter2Motor);
  }
}