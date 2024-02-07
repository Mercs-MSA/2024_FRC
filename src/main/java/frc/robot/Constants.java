package frc.robot;


import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;


public final class Constants {
    public static final double stickDeadband = 0.1;


    public static final class Swerve {
        public static final int pigeonID = 16;


        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X2_12);


        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.2);
        public static final double wheelBase = Units.inchesToMeters(20.2);
        public static final double wheelCircumference = chosenModule.wheelCircumference;


        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));


        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;


        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;


        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;


        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;


        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;


        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;


        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;


        /* Drive Motor PID Values */
        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;


        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;


        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;


        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;


         /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 39;
            public static final int angleMotorID = 40;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-88.5);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }


        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 36;
            public static final int angleMotorID = 37;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-94.4);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
       
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 33;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(51.1);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }


        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 34;
            public static final int angleMotorID = 35;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-69.8+180);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }


    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
   
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
   
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    
    public static class IntakeConstants {
        public static final int kIntakeMotorId = 26;
        public static final double kIntakeMotorSpeed = 0.30;
        public static final int kIndexMotorId = 24;
        public static final double kIndexMotorSpeed = 0.10;
        public static final int kIntakeSensorId = 1;
    }

    // This is for miscellaneous constants
    public static class Misc {
        public static final double Conversion_Factor = 6.3648;    // this is the calculated conversion factor at 19.4 volts (nominal expected value)
    }

    public static class SATConstants {


        // Motor IDs
        public static final int SAT_SHOOTER1_MOTOR_ID = 29;
        public static final int SAT_SHOOTER2_MOTOR_ID = 27;


        public static final int SAT_PIVOT_MOTOR_ID = 19;
        public static final int SAT_BASE1_MOTOR_ID = 55;
        public static final int SAT_BASE2_MOTOR_ID = 25;
        // INDEXER IS 24


        public static final int SAT_OBJECTDETECTOR_SENSOR_ID = 1;


        public static final int SAT_SERVO1_SERVO_ID = 9;
        public static final int SAT_SERVO2_SERVO_ID = 8;
        public static final int SAT_SERVO3_SERVO_ID = 7;
        public static final int SAT_SERVO4_SERVO_ID = 6;
        /*lift start positions */
        public static final double BASE_START_POS = 0.0;
        public static final double PIVOT_START_POS = 0.0;
        /*podium scoring position */
        public static final double BASE_PODIUM_POS = 3.717;
        public static final double PIVOT_PODIUM_POS = 0.0;
        /*SUBWOOFER scoring position */
        public static final double BASE_SUB_POS = 7.026;
        public static final double PIVOT_SUB_POS = 0.0;
        /*AMP scoring position */
        public static final double BASE_AMP_POS = 5.0;
        public static final double PIVOT_AMP_POS = 22.0;
        /*TRAP scoring position */
        public static final double BASE_TRAP_POS = 13.803;
        public static final double PIVOT_TRAP_POS = 41.665;

        public static final double SHOOTER_SPEED = 0.6;


        public static final int BASE_THROUGHBORE_ENCODER = 2;
        public static final int PIVOT_THROUGHBORE_ENCODER = 3;
        /*THIS NEEDS TO INCLUDE THE CONVERSION FROM DEGREES TO ENCODER COUNTS */
        public static final int BASE_ENCODER_RATIO = 50;
        public static final int PIVOT_ENCODER_RATIO = 20;


      // PID coefficients
        public static final double kP = 0.1;
        public static final double kI = 1e-4;
        public static final double kD = 1;
        public static final double kIz = 0;
        public static final double kFF = 0;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;


        }
       
    public static final class climberConstants
    {
 
      public static final int tubeMotor_Left_ID = 9;    ///  reassign this to something else...
      public static final int tubeMotor_Right_ID = 3;


      public static final int climber_Increment = -100;


      public static final int bottom_climber_position = 0;
      public static final int test_climber_position = -1000;
      public static final int test2_climber_position = -500;
    }

    public static final class State {
        public static robotState currentRobotState = robotState.IDLE;


        enum robotState{
            INTAKE, //the intake must be at ground note pickup position, the SAT must be flat, the note is stored in intake (for now)
            IDLE, //the intake must be up, the SAT must be flat, the note is stored in feeder (part of SAT)
            PIVOT, //the intake must be up, the SAT will move to angled position based on kinematics calculations, the note is stored in feeder (part of SAT)
            SCORING //the intake must be up, the SAT must be at thge angled position, the note must move from feeders to flywheels  
        }  
   
        // if intake IR does not detect note, the intake must keep spining; else, intake does not spin
        // flywheel must always be spinning
        // if robot state is SCORING, feeder must be spinning; else, feeder does not spin
        // Big Question: can we combine IDLE and PIVOT??? (ans: yes, these will be done auto without driver from switching states)


        public static robotState getState(){
            return currentRobotState;
        }


        public static void setState(String newState) {
            switch (newState.toUpperCase()) {
                case "INTAKE":
                    currentRobotState = robotState.INTAKE;
                    break;
                case "IDLE":
                    currentRobotState = robotState.IDLE;
                    break;
                case "PIVOT":
                    currentRobotState = robotState.PIVOT;
                    break;
                case "SCORING":
                    currentRobotState = robotState.SCORING;
                    break;
                default:
                    currentRobotState = robotState.INTAKE; //do I want to do this?
                    break;
            }
        }
       
    }
}



