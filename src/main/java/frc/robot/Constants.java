package frc.robot;


import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
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
        public static final double kIntakeMotorSpeed = 0.4;
        public static final double kIntakeProcessRotations = 1.9;
        public static final double kIndexProcessRotations = 2.0;
        public static final int kIndexMotorId = 24;
        public static final double kIndexMotorSpeed = 0.70;
        public static final int kIntakeLowerSensorId = 1;
        public static final int kIntakeUpperSensorId = 0;
        public static final int kIntakeLowerSensor2Id = 2;
        public static final int kIntakeUpperSensor3Id = 3;       
        public static final double kIntakeMotorTolerance = 0.3;
        public static final double kIndexMotorTolerance = 0.3;
        public static final double kIntakeMotorDCTolerance = 0.1;
        public static final double kIndexMotorDCTolerance = 0.1;
        
        /*
         * States for Intake:
         * IDLE: the motor is off, sensor doesn't see anything, doesn't have a note in it
         * START: the motor on, sensor doesn't see anything
         * INTAKE: the motor is on, the sensor sees a note enter
         * PROCESS: the motor is on but moving precisely, the sensor doesn't see anything
         * HOLD: the motor is off, the sensor doesn't see anything, has a note in it
         * INDEX: the motor is on, the sensor sees a note leave
         */
        public static intakeState currentIntakeState = intakeState.IDLE;
        public enum intakeState{
            IDLE, 
            START, 
            INTAKE,
            PROCESS, 
            HOLD,
            INDEX,
            OVERRIDE_MOTOR_ON,
            OVERRIDE_MOTOR_OFF
        }
        /*
         * States for Index:
         * IDLE: the motor is off, sensor doesn't see anything, doesn't have a note in it
         * START: the motor is on, sensor doesn't see anything
         * INTAKE: the motor is on, the sensor sees a note enter
         * PROCESS: the motor is on but moving precisely, the sensor doesn't see anything
         * HOLD: the motor is off, the sensor doesn't see anything, has a note in it
         * FIRE: the motor is on, the sensor sees the note leave
         */
        public static indexState currentIndexState = indexState.IDLE;
        public enum indexState{
            IDLE, 
            START, 
            INTAKE,
            PROCESS, 
            HOLD,
            FIRE,
            OVERRIDE_MOTOR_ON,
            OVERRIDE_MOTOR_OFF
        }    
   
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

        /*lift start positions */
        public static final double MOTOR1_BASE_START_POS = 0.155;
        public static final double MOTOR2_BASE_START_POS = 0.342;
        public static final double PIVOT_START_POS = 0.1;
        /*podium scoring position */
        public static final double MOTOR1_BASE_PODIUM_POS = -1.575;
        public static final double MOTOR2_BASE_PODIUM_POS = -1.723;
        public static final double PIVOT_PODIUM_POS = 0.4;
        /*SUBWOOFER scoring position */
        public static final double MOTOR1_BASE_SUB_POS = -18.5268;
        public static final double MOTOR2_BASE_SUB_POS = -18.76;
        public static final double PIVOT_SUB_POS = 1.7;
        /*AMP scoring position */
        public static final double MOTOR1_BASE_AMP_POS = -16.16;
        public static final double MOTOR2_BASE_AMP_POS = -16.26;
        public static final double PIVOT_AMP_POS = 33.8;
        /*TRAP scoring position */
        public static final double MOTOR1_BASE_TRAP_POS = 0.2;
        public static final double MOTOR2_BASE_TRAP_POS = 0.4;
        public static final double PIVOT_TRAP_POS = 0.1;

        public static final double PIVOT_MECHANICALLY_REQUIRED_POS = 7.8;
        public static final double MOTOR_TOLERANCE = 0.5;

        public static final double SHOOTER_SPEED = 70;
        public static final double kShooterSpeedTolerance = 1.0;

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

        public static Position state = Position.START;
        public static enum Position {
            PODIUM,
            SUB,
            AMP,
            TRAP,
            START, 
            MOVING
          }
    }
       
    public static final class climberConstants
    {
 
      public static final int tubeMotor_Left_ID = 23;    ///  reassign this to something else...
      public static final int tubeMotor_Right_ID = 38;


      public static final double climber_Increment = 0.5;

      public static final double LEFT_BOTTOM_POSITION = 4.33;
      public static final double RIGHT_BOTTOM_POSITION = -10.71;

      public static final double LEFT_TOP_POSITION = -135;
      public static final double RIGHT_TOP_POSITION = 126;


      public static final double LEFT_MID_POSITION = -60;
      public static final double RIGHT_MID_POSITION = 51;
    
    }

    public static class Vision {
        public static boolean isNoteDetected = false;

        public static class aprilTagBackLeft {
            public static String camera = "AprilTagBackLeft";
            public static Transform3d robotToCamera = new Transform3d(-0.1820926, -0.2952496, 0.3039618, new Rotation3d(0, 0.785398, -0.785398));
        }
        
        public static class aprilTagFrontRight {
            public static String camera = "AprilTagFrontRight";
            public static Transform3d robotToCamera = new Transform3d(-0.2269744, -0.2446782, 0.3039618, new Rotation3d(0, -0.785398, -0.785398));
        }
    }

    public static final class State {
        public static robotState currentRobotState = robotState.IDLE;


        enum robotState{
            INTAKE, //the intake must be at ground note pickup position, the SAT must b flat, the note is stored in intake (for now)
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



