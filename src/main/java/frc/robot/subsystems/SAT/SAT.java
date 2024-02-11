// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SAT;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

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
  private final VelocityVoltage satShooter1_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage satPivotMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage satBase1_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage satBase2_voltagePosition = satBase1_voltagePosition.clone();

  double base1MotorPos, base2MotorPos, pivotMotorPos, shooterMotorSpeed, Base1StartPosition, Base2StartPosition, PivotStartPosition;
  double baseTargetPose, pivotTargetPose = 0.0;

  TalonFXConfiguration satBase1MotorConfigs, satBase2MotorConfigs;

  // private final DigitalInput satObjectDectecter = new DigitalInput(Constants.SATConstants.SAT_OBJECTDETECTOR_SENSOR_ID);

  // Peak output of 8 volts

  public SAT() {
    /**
     * this stuff happens ONCE, when the code enables, NOT WHEN THE ROBOT ENABLES
     */
    satBase1MotorConfigs = new TalonFXConfiguration();
    satBase1MotorConfigs.Slot0.kP = 1.0; // An error of 0.5 rotations results in 1.2 volts output
    satBase1MotorConfigs.Slot0.kD = 0; // A change of 1 rotation per second results in 0 volts output
    satBase1MotorConfigs.Slot0.kG = 0.0;
    satBase1MotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Peak output of 8 volts
    satBase1MotorConfigs.Voltage.PeakForwardVoltage = 14;
    satBase1MotorConfigs.Voltage.PeakReverseVoltage = -14;
    satBase1MotorConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    satBase1MotorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    satBase2MotorConfigs = new TalonFXConfiguration();
    satBase2MotorConfigs.Slot0.kP = 1.0; // An error of 0.5 rotations results in 1.2 volts output
    satBase2MotorConfigs.Slot0.kD = 0; // A change of 1 rotation per second results in 0 volts output
    satBase2MotorConfigs.Slot0.kG = 0.0;
    satBase2MotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Peak output of 8 volts
    satBase2MotorConfigs.Voltage.PeakForwardVoltage = 14;
    satBase2MotorConfigs.Voltage.PeakReverseVoltage = -14;
    satBase2MotorConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    satBase2MotorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;


    TalonFXConfiguration satPivotMotorConfigs = new TalonFXConfiguration();
    satPivotMotorConfigs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    satPivotMotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    satPivotMotorConfigs.Voltage.PeakForwardVoltage = 8;
    satPivotMotorConfigs.Voltage.PeakReverseVoltage = -8;

    TalonFXConfiguration satShooter1MotorConfigs = new TalonFXConfiguration();
    satShooter1MotorConfigs.Slot0.kP = 0.6; // An error of 0.5 rotations results in 1.2 volts output
    satShooter1MotorConfigs.Slot0.kS = 0.05; // Add 0.05 V output to overcome static friction
    satShooter1MotorConfigs.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    satShooter1MotorConfigs.Slot0.kI = 0; // no output for integrated error
    satShooter1MotorConfigs.Slot0.kD = 0; // no output for error derivative

    satShooter1MotorConfigs.Voltage.PeakForwardVoltage = 14;
    satShooter1MotorConfigs.Voltage.PeakReverseVoltage = -14;
    satShooter1MotorConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
    satShooter1MotorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.4;  

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

    //satBase1Motor.setPosition(0);
    //satBase2Motor.setPosition(0);
    //satPivotMotor.setPosition(0);

    Base1StartPosition = satBase1Motor.getPosition().getValueAsDouble();
    Base2StartPosition = satBase2Motor.getPosition().getValueAsDouble();
    PivotStartPosition = satPivotMotor.getPosition().getValueAsDouble();

    PhysicsSim.getInstance().addTalonFX(satBase1Motor, 0.001);
    PhysicsSim.getInstance().addTalonFX(satBase2Motor, 0.001);
    PhysicsSim.getInstance().addTalonFX(satPivotMotor, 0.001);
    PhysicsSim.getInstance().addTalonFX(satShooter1Motor, 0.001);
    PhysicsSim.getInstance().addTalonFX(satShooter2Motor, 0.001);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    base1MotorPos = satBase1Motor.getPosition().getValueAsDouble();
    base2MotorPos = satBase2Motor.getPosition().getValueAsDouble();
    pivotMotorPos = satPivotMotor.getPosition().getValueAsDouble();
    shooterMotorSpeed = satShooter1Motor.getVelocity().getValueAsDouble();

    SmartDashboard.putNumber("base1MotorPos", base1MotorPos);
    SmartDashboard.putNumber("base2MotorPos", base2MotorPos);

    SmartDashboard.putNumber("base1PosRelativeToStart", base1MotorPos - Base1StartPosition);
    SmartDashboard.putNumber("base2PosRelativeToStart", base2MotorPos - Base2StartPosition);

    SmartDashboard.putNumber("base1 command", satBase1Motor.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putNumber("base2 command", satBase2Motor.getClosedLoopReference().getValueAsDouble());

    SmartDashboard.putNumber("base1MotorVoltage", satBase1Motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("base2MotorVoltage", satBase2Motor.getMotorVoltage().getValueAsDouble());

    SmartDashboard.putNumber("pivotMotorPos", pivotMotorPos);
    SmartDashboard.putNumber("shooter speed", shooterMotorSpeed);
    SmartDashboard.putNumber("shooter command", satShooter1Motor.getClosedLoopReference().getValueAsDouble());

    SmartDashboard.putNumber("Base1 Motor Temperature", satBase1Motor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Base2 Motor Temperature", satBase2Motor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Motor Temperature", satPivotMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Motor1 Temperature", satShooter1Motor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Motor2 Temperature", satShooter2Motor.getDeviceTemp().getValueAsDouble());

  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
  
  public void goToBasePodiumPosition() {
    satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.MOTOR1_BASE_PODIUM_POS));
    satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.MOTOR2_BASE_PODIUM_POS));
  }

  /// This is for test purposes only
  public void baseGoToPosition(double increment) {
    baseTargetPose = baseTargetPose + (increment);
    satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Base1StartPosition + baseTargetPose));
    satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Base2StartPosition + baseTargetPose));
  }

  /// This is for test purposes only
  public void pivotGoToPosition(double increment) {
    pivotTargetPose = pivotTargetPose + (increment);
    satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(PivotStartPosition + pivotTargetPose));
  }

  public void goToBaseSubPosition() {
    satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.MOTOR1_BASE_SUB_POS));
    satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.MOTOR2_BASE_SUB_POS));    
  }

  public void goToBaseAmpPosition() {
    satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.MOTOR1_BASE_AMP_POS));
    satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.MOTOR2_BASE_AMP_POS));
  }

  public void goToBaseTrapPosition() {
    satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.MOTOR1_BASE_TRAP_POS));
    satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.MOTOR2_BASE_TRAP_POS));
  }

  public void goToBaseZeroPosition() {
    satBase1Motor.setControl(satBase1_voltagePosition.withPosition(Constants.SATConstants.MOTOR1_BASE_START_POS));
    satBase2Motor.setControl(satBase2_voltagePosition.withPosition(Constants.SATConstants.MOTOR2_BASE_START_POS));
  }

   public void goToPivotPodiumPosition() {
    satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(Constants.SATConstants.PIVOT_PODIUM_POS));
  }

   public void goToPivotSubPosition() {
    satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(Constants.SATConstants.PIVOT_SUB_POS)); 
  }

   public void goToPivotAmpPosition() {
    satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(Constants.SATConstants.PIVOT_AMP_POS));    
  }

   public void goToPivotTrapPosition() {
    satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(Constants.SATConstants.PIVOT_TRAP_POS));    
  }

   public void goToPivotZeroPosition() {
    satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(Constants.SATConstants.PIVOT_START_POS)); 
  }

  public void shootNote() { 
    satShooter1Motor.setControl(satShooter1_voltageVelocity.withVelocity(Constants.SATConstants.SHOOTER_SPEED)); 
  }

  public void stopShooter() {
    satShooter1Motor.setControl(new NeutralOut());
  }

  public boolean isWithinTol(double targetPose, double currentPose, double tolerance){
    if (Math.abs(targetPose - currentPose) <= tolerance){
        return true;
    }
    else {
        return false;
    }
  }

  public double getBase1Pos() {
    return base1MotorPos;
  }

  public double getBase2Pos() {
    return base2MotorPos;
  }

  public double getPivotPos() {
    return pivotMotorPos;
  }

  public double getShooterSpeed() {
    return shooterMotorSpeed;
  }

  public enum BaseMotorsPosition {
    BASE_PODIUM_POS,
    BASE_SUB_POS,
    BASE_AMP_POS,
    BASE_TRAP_POS,
    BASE_START_POS
  }
}