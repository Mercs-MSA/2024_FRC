package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

//  USE NEXT LINE FOR TESTING
import frc.robot.sim.PhysicsSim;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

        // expanded is there to support current data for troubleshooting
        optimization_for_CAN();
        expanded_diagnostic_data();
        

        // USE NEXT LINE FOR TESTING
        PhysicsSim.getInstance().addTalonFX(mAngleMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(mDriveMotor, 0.001);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public double getDriveCurrent(){
        return (mDriveMotor.getStatorCurrent().getValue());
    }

    public double getAngleCurrent(){
        return (mAngleMotor.getStatorCurrent().getValue());
    }

    public void expanded_diagnostic_data() {
        StatusSignal<Double> m_AngleMotorCurrent_canbus1signal4 = mAngleMotor.getSupplyCurrent();
        StatusSignal<Double> m_DriveMotorCurrent_canbus1signal5 = mDriveMotor.getSupplyCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(60, m_AngleMotorCurrent_canbus1signal4, m_DriveMotorCurrent_canbus1signal5);
    }

    public void optimization_for_CAN() {
        StatusSignal<Double> m_AngleEncoder_canbus1signal1 = angleEncoder.getAbsolutePosition();
        StatusSignal<Double> m_AngleMotor_canbus1signal2 = mAngleMotor.getPosition();
        StatusSignal<Double> m_DriveMotor_canbus1signal3 = mDriveMotor.getPosition();
        StatusSignal<Double> m_AngleMotor_canbus1signal4 = mAngleMotor.getMotorVoltage();
        StatusSignal<Double> m_DriveMotor_canbus1signal5 = mDriveMotor.getMotorVoltage();
        m_AngleEncoder_canbus1signal1.setUpdateFrequency(1);
        BaseStatusSignal.setUpdateFrequencyForAll(60, m_AngleMotor_canbus1signal2, m_DriveMotor_canbus1signal3, m_AngleMotor_canbus1signal4, m_DriveMotor_canbus1signal5);
        angleEncoder.optimizeBusUtilization();
        ParentDevice.optimizeBusUtilizationForAll(mDriveMotor, mAngleMotor);
    }
}