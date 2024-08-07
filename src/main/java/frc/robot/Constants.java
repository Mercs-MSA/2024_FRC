package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;


public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 16;


        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X2_10);


        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.431802);
        public static final double wheelBase = Units.inchesToMeters(19.431802);
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
        public static final int angleCurrentLimit = 15;
        public static final int angleCurrentThreshold = 25;
        public static final double angleCurrentThresholdTime = 0.05;
        public static final boolean angleEnableCurrentLimit = true;


        public static final int driveCurrentLimit = 30;
        public static final int driveCurrentThreshold = 35;
        public static final double driveCurrentThresholdTime = 0.05;
        public static final boolean driveEnableCurrentLimit = true;


        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.3;
        public static final double closedLoopRamp = 0.2;


        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;


        /* Drive Motor PID Values */
        public static final double driveKP = 0.25;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;


        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;


        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.0;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10;


        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;


         /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 39;
            public static final int angleMotorID = 40;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(62.4);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }


        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 36;
            public static final int angleMotorID = 37;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(83.232421875 + 180); //-98.525390625 or //83.232421875 - 180
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
       
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 33;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(48.5);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }


        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 34;
            public static final int angleMotorID = 35;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-73.037109375 + 180);
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
        public static final double kIntakeMotorSpeed = 100;
        public static final double kIntakeProcessRotations = 1.9;
        public static final double kIndexProcessRotations = -1.5;
        public static final int kIndexMotorId = 27; 
        public static final int kIndexMotorSpeed = 90;
        public static final double kSlowIndexMotorSpeed = -5;
        public static final int kIntakeUpperSensorId = 0;
        public static final int kIntakeLowerSensor1Id = 1;
        public static final int kIntakeLowerSensor2Id = 2;
        public static final int kIntakeLowerSensor3Id = 3;       
        public static final double kIntakeMotorTolerance = 0.3;
        public static final double kIndexMotorTolerance = 0.1;
        public static final double kIntakeMotorDCTolerance = 0.3;
        public static final double kIndexMotorDCTolerance = 0.05;

        public static boolean kRobotHasNote;
        
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
        public static final int SAT_SHOOTER_LEFT_MOTOR_ID = 23;
        public static final int SAT_SHOOTER_RIGHT_MOTOR_ID = 24;

        public static final int SAT_PIVOT_MOTOR_ID = 19;
        public static final int SAT_BASE1_MOTOR_ID = 55;
        public static final int SAT_BASE2_MOTOR_ID = 25;
        // INDEXER IS 24

        public static final int SAT_OBJECTDETECTOR_SENSOR_ID = 1;
        
        public static class SUBWOOFER{
            public static final double motor1_base = -9.571777;
            public static final double motor2_base = -7.906;
            public static final double pivot = 0.0;
            public static final double shooterSpeed = -35;
        }

        public static class AMP_STAGE_1{
            public static final double motor1_base = -15;
            public static final double motor2_base = -15;
            public static final double pivot = -70;
            public static final double shooterSpeed = 0;
        }

        public static class AMP_STAGE_3{
            public static final double motor1_base = -35.9858;
            public static final double motor2_base = 0;
            public static final double pivot = -142.5815;
            public static final double shooterSpeed = -10;
        }

        public static class AMP_STAGE_2{
            public static final double motor1_base = -25;
            public static final double motor2_base = 0;
            public static final double pivot = -100;
            public static final double shooterSpeed = -10;

        }

        public static class TRAP{
            public static final double motor1_base = START.motor1_base;
            public static final double motor2_base = START.motor2_base;
            public static final double pivot = 0.0; //46.939;
            public static final double shooterSpeed = -20;
        }

        public static class PODIUM{
            public static final double motor1_base = START.motor1_base;
            public static final double motor2_base = START.motor2_base;
            public static final double pivot = -16;
            public static final double shooterSpeed = -50;
        }

        public static class WING{
            public static final double motor1_base = -9.571777;
            public static final double motor2_base = -7.906;
            public static final double pivot = 0.0;
            public static final double shooterSpeed = -20.5;
        }

        public static class HANDOFF{
            public static final double motor1_base = START.motor1_base;
            public static final double motor2_base = START.motor2_base;
            public static final double pivot = -2.5;
            public static final double shooterSpeed = 0;
        }

        public static class START{
            public static final double motor1_base = 0.334;
            public static final double motor2_base = -0.334;
            public static final double pivot = -1.21;
            public static final double shooterSpeed = 0;
        }

        public static final double PIVOT_MECHANICALLY_REQUIRED_POS = 7.8;
        public static final double MOTOR_TOLERANCE = 0.6;

        public static final double SHOOTER_SPEED = -50;
        public static final double kShooterSpeedTolerance = 8.0;

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
            WING,
            HANDOFF
        }

        public static void setState(String pos){
            pos = pos.toLowerCase();
            switch (pos) {
                case "podium":
                    state = Position.PODIUM;
                    break;
                case "sub":
                    state = Position.SUB;
                    break;
                case "amp":
                    state = Position.AMP;
                    break;
                case "trap":
                    state = Position.TRAP;
                    break;
                case "wing":
                    state = Position.WING;
                    break;
                case "handoff":
                    state = Position.HANDOFF;
                    break;
                case "start":
                    state = Position.START;
                    break;
                default:
                    state = Position.START;
                    break;
            }
        }

        public String getState(){
            return state.toString().toLowerCase();
        }
    }
       
    public static final class climberConstants
    {
 
      public static final int tubeMotor_Left_ID = 38;    ///  reassign this to something else...
      public static final int tubeMotor_Right_ID = 23;


      public static final double climber_Increment = 64;

      public static final double LEFT_BOTTOM_POSITION = -1.10;
      public static final double RIGHT_BOTTOM_POSITION = 0.80;

      public static final double LEFT_TOP_POSITION = -146.0;
      public static final double RIGHT_TOP_POSITION = -146.0;


      public static final double LEFT_MID_POSITION = -70.0;
      public static final double RIGHT_MID_POSITION = -70.0;
    
    
    }

    public static class Vision {
        public static boolean isNoteDetected = false;

        public static boolean visionTurnedOn = false;

        public static boolean isRedAlliance = false;

        public static double gamePieceYawOffset = -56.088;

        public static class gamePieceCameraInfo {
            // public static Transform3d robotToCamera = new Transform3d(12.114, -8.874, 15.266, new Rotation3d(0, 0, 0));
            // public static Transform3d robotToCamera = new Transform3d(10, -3.5, 16.966, new Rotation3d(0, 0, 0));
            public static Transform3d robotToCamera = new Transform3d(10, -3.5, 16.966, new Rotation3d(0, 0, 0));
        }

        public static Pose2d temp = new Pose2d();

        public static class aprilTagBackLeft {
            public static String camera = "AprilTagBackLeft";
            public static Transform3d robotToCamera = new Transform3d(-Units.inchesToMeters(9.647), Units.inchesToMeters(8.923), Units.inchesToMeters(1.7 + 9.217), new Rotation3d(0, 0.785398, -0.785398));
        }
        
        public static class aprilTagFrontRight {
            public static String camera = "AprilTagFrontRight";
            public static Transform3d robotToCamera = new Transform3d(Units.inchesToMeters(5.668), -Units.inchesToMeters(10.631), Units.inchesToMeters(1.7 + 17.470), new Rotation3d(0, -0.785398, -0.785398));
        }

        public static class gamePieceCamera {
            public static String camera = "GamePieceCamera";
            public static Transform3d robotToCamera = new Transform3d(Units.inchesToMeters(5.537), -Units.inchesToMeters(-10.120), Units.inchesToMeters(1.7 + 15.216), new Rotation3d(0, 0, 0));
        }

        public static double getRobotHeading(double gamePieceYaw){
            return ((gamePieceYaw*0.501) + 11.3);
        }

        public static class Sub{
            public static Pose2d bluePose = new Pose2d(new Translation2d(1.38, 5.54), Rotation2d.fromDegrees(0));
            public static Pose2d redPose = convertToRedSide(bluePose);
        }

        public static class SubRight{
            public static Pose2d bluePose = new Pose2d(new Translation2d(0.75, 4.48), Rotation2d.fromDegrees(-60));
            public static Pose2d redPose = convertToRedSide(bluePose);
        }

        public static class Podium{
            public static Pose2d bluePose = new Pose2d(new Translation2d(2.977, 4.082), Rotation2d.fromDegrees(-32.61));
            public static Pose2d redPose = convertToRedSide(bluePose);
        }

        public static double fieldWidth = 16.541;
        public static double fieldHeight = 8.211;

        public static final Pose2d convertToRedSide(Pose2d pose) {
            return new Pose2d(fieldWidth - pose.getX(), pose.getY(), Rotation2d.fromDegrees(180).minus(pose.getRotation()));

        }

        public static Pose2d getPose(String a){
            a = a.toLowerCase();
            if (a == "sub"){
                if (isRedAlliance){
                    return Sub.redPose;
                }
                return Sub.bluePose;
            }
            else if (a == "podium"){
                if (isRedAlliance){
                    return Podium.redPose;
                }
                return Podium.bluePose;
            }
            else if (a == "subright"){
                if (isRedAlliance){
                    return SubRight.redPose;
                }
                return SubRight.bluePose;
            }

            return null;
        }

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        
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

    public static final class ScoringConstants {
        public static ScoringMode currentScoringMode = ScoringMode.SUBWOOFER;
        public enum ScoringMode {
            WING,
            AMP1,
            AMP2,
            SUBWOOFER,
            PODIUM,
            TRAP
        }
    }

    public static boolean isWithinTol(double targetPose, double currentPose, double tolerance) {
        return (Math.abs(targetPose - currentPose) <= tolerance);
      }
}



