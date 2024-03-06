package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class CustomGamePieceVision extends SubsystemBase{
    private NetworkTableInstance ntInst;
    private DoubleTopic yawTopic;
    private DoubleTopic distTopic;
    private DoubleSubscriber yawSubscriber;
    private DoubleSubscriber distSubscriber;

    boolean visionOk;

    double gamePieceYaw = 999.0;
    double gamePieceDist = 999.0;

    public CustomGamePieceVision(String yawString, String distString){
        ntInst = NetworkTableInstance.getDefault();

        // get a topic from a NetworkTableInstance
        // the topic name in this case is the full name
        this.yawTopic = ntInst.getDoubleTopic("/Vision/" + yawString);
        this.distTopic = ntInst.getDoubleTopic("/Vision/" + distString);

        if (this.yawTopic == null || this.distTopic == null) {
            visionOk = false;
            return;
        } else {
            visionOk = true;
        }

        this.yawSubscriber = this.yawTopic.subscribe(999.0);
        this.distSubscriber = this.distTopic.subscribe(999.0);
    }

    @Override
    public void periodic(){
        if (visionOk) {
            gamePieceYaw = yawSubscriber.get();
            gamePieceDist = distSubscriber.get();
            SmartDashboard.putNumber("Game Piece Yaw", gamePieceYaw);
            SmartDashboard.putNumber("Game Piece Yaw - Offset", gamePieceYaw - Constants.Vision.gamePieceYawOffset);
            SmartDashboard.putNumber("Game Piece Dist", gamePieceDist);
            SmartDashboard.putNumber("Note Bounding Box Width", convertDistanceToWidth());
            SmartDashboard.putNumber("Note Width Angle", convertWidthToAngle());  
            SmartDashboard.putNumber("Note estimated distance from Pupil to center", convertNoteAngleToDistance());     
            SmartDashboard.putNumber("Calculated Robot Command Yaw (new version)", calculateGamePieceHeading2());
            SmartDashboard.putNumber("Calculated Robot Command Yaw (v3)", calculateGamePieceHeading3());     
            SmartDashboard.putNumber("Calculated Robot Command Yaw", calculateGamePieceHeading());
            Constants.Vision.isNoteDetected = (gamePieceYaw != 999.0);
        }
    }

    /**
     * Gets the Yaw angle of the game piece.
     * @return The Yaw angle.
     */
    public double getGamePieceYaw(){
        return gamePieceYaw;
    }

    /**
     * Gets the distance of the game piece.
     * @return The Distance (inches)
     */
    public double getGamePieceDist(){
        return gamePieceDist;
    }


    /**
     * Align center of camera with game piece
     * @param Swerve drive subsystem
     */
    public void alignNoteYaw(Swerve swerve) {
        if (visionOk) {
            swerve.drive(new Translation2d(0, 0), gamePieceYaw/20, false, true);
        }
    }

     /**
     * Generate yaw command based on Note sighting from the camera
     * Output is robot rotation in radians (relative to the current heading) to turn towards the target
     */
    public double calculateGamePieceHeading() {
        double xFromCenter = -1*(gamePieceDist * Math.sin(Math.toRadians(gamePieceYaw)) + Constants.Vision.gamePieceCameraInfo.robotToCamera.getX());
        double yFromCenter = (gamePieceDist * Math.cos(Math.toRadians(gamePieceYaw)) - Constants.Vision.gamePieceCameraInfo.robotToCamera.getY());
        SmartDashboard.putNumber("xFromCenter", xFromCenter);
        SmartDashboard.putNumber("yFromCenter", yFromCenter); 
        return Math.toDegrees(Math.atan(yFromCenter/xFromCenter));
    }

     /**
     * Another Try at this...    1280 x 720 camera; 110 degree FOV, so horizontal IFOV is ~ 0.086 degrees per pixel.
     * https://pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
     * 46" below had focal length of 345
     * 44" below and left, had focal length of 370.9
     * 
     */
    public double calculateGamePieceHeading2() {
        double groundNoteAngle = Math.acos(Constants.Vision.gamePieceCameraInfo.robotToCamera.getZ()/convertNoteAngleToDistance()); //angle of gamepiece distance and height of camera
        double trueHypotenuse = Math.sin(groundNoteAngle) * convertNoteAngleToDistance();
        double xFromCenter = (trueHypotenuse * Math.sin(Math.toRadians(gamePieceYaw)) + Constants.Vision.gamePieceCameraInfo.robotToCamera.getX());
        double yFromCenter = (trueHypotenuse * Math.cos(Math.toRadians(gamePieceYaw)) - Constants.Vision.gamePieceCameraInfo.robotToCamera.getY());
        return Math.toDegrees(Math.atan(yFromCenter/xFromCenter));
    }

    public double calculateGamePieceHeading3() {
        double xFromCenter = (convertBoxWidthToDistance2() * Math.sin(Math.toRadians(-1*gamePieceYaw)) + Constants.Vision.gamePieceCamera.robotToCamera.getX());
        double yFromCenter = (convertBoxWidthToDistance2() * Math.cos(Math.toRadians(-1*gamePieceYaw)) + Constants.Vision.gamePieceCamera.robotToCamera.getY());
        return Math.toDegrees(Math.atan(yFromCenter/xFromCenter));
    }

     /**
     * Convert the game piece camera bounding box into an estimated distance based on calculated focal length of 360
     */
    public double convertBoxWidthToDistance2() {
        return (360*14/(14*288.14/(gamePieceDist)));
    }

     /**
     * Convert the game piece camera distance info from gamevision back into the bounding box width
     */
    public double convertDistanceToWidth() {
        return 14*288.14/(gamePieceDist);
    }

     /**
     * Convert the game piece camera bounding box width to an angular extent in degrees
     * Angle of view = (180/π) × 2 × aTan(Image size / (2 × Focal length × (Magnification + 1))),
     */
    public double convertWidthToAngle() {
        return convertDistanceToWidth()*0.058333;
    }

     /**
     * Determine the Note Distance based on its angular extent in the FOV
     * Split in into 2 right triangles and you can calculate the distance using width angle divided by 2, and known Note wisth of 14 inches (divide by 2)
     */
    public double convertNoteAngleToDistance() {
        return 7.0/Math.tan(0.5*Units.degreesToRadians(convertWidthToAngle()));
    }
}
