// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.sim.PhysicsSim;

public class Pivot extends SubsystemBase {
  private final TalonFX satPivotMotor = new TalonFX(SATConstants.SAT_PIVOT_MOTOR_ID);
  private final PositionVoltage satPivotMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

  private double pivotMotorPos, pivotStartPosition;

    public Pivot() {

        TalonFXConfiguration satPivotMotorConfigs = new TalonFXConfiguration();
        satPivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        satPivotMotorConfigs.Slot0.kP = 2; // An error of 0.5 rotations results in 1.2 volts output
        satPivotMotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
        // Peak output of 8 volts
        satPivotMotorConfigs.Voltage.PeakForwardVoltage = 14;
        satPivotMotorConfigs.Voltage.PeakReverseVoltage = -14;

        // STATUS FOR PIVOT
        StatusCode status1 = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status1 = satPivotMotor.getConfigurator().apply(satPivotMotorConfigs);
            if (status1.isOK())
            break;
        }
        if (!status1.isOK()) {
            System.out.println("Could not apply configs, error code: " + status1.toString());
        }

        pivotStartPosition = satPivotMotor.getPosition().getValueAsDouble();

        optimization_for_CAN();


        PhysicsSim.getInstance().addTalonFX(satPivotMotor, 0.001);
    }

    @Override
    public void periodic() {
        pivotMotorPos = satPivotMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("pivotMotorPos", pivotMotorPos);
        SmartDashboard.putNumber("pivotRelativeMotorPos", pivotMotorPos - pivotStartPosition);
        SmartDashboard.putNumber("Pivot Motor Temperature", satPivotMotor.getDeviceTemp().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    public void pivotGoToPositionIncrement(double increment) {
        satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(this.pivotMotorPos + increment*6));
    }

    public void movePivotMotor(double pos){
        satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(pos));
    }

    public double getPivotPos() {
        return pivotMotorPos;
    }

    public void resetMotors(){
        satPivotMotor.setControl(new NeutralOut());
    }

    public void goToHomePos(){
        satPivotMotor.setControl(satPivotMotor_voltagePosition.withPosition(Constants.SATConstants.START.pivot));
    }

    public void optimization_for_CAN() {
        StatusSignal<Double> m_PivotMotor_canbus1signal1 = satPivotMotor.getPosition();
        StatusSignal<Double> m_PivotTemp_canbus1signal6 = satPivotMotor.getDeviceTemp();
        BaseStatusSignal.setUpdateFrequencyForAll(60, m_PivotMotor_canbus1signal1);
        BaseStatusSignal.setUpdateFrequencyForAll(1, m_PivotTemp_canbus1signal6);
        ParentDevice.optimizeBusUtilizationForAll(satPivotMotor);
    }
}