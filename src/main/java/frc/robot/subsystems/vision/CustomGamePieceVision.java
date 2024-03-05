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
        double groundNoteAngle = Math.acos(Constants.Vision.gamePieceCameraInfo.robotToCamera.getZ()/gamePieceDist); //angle of gamepiece distance and height of camera
        double trueHypotenuse = Math.sin(groundNoteAngle) * gamePieceDist;
        double xFromCenter = (trueHypotenuse * Math.sin(Math.toRadians(gamePieceYaw)) + Constants.Vision.gamePieceCameraInfo.robotToCamera.getX());
        double yFromCenter = (trueHypotenuse * Math.cos(Math.toRadians(gamePieceYaw)) - Constants.Vision.gamePieceCameraInfo.robotToCamera.getY());
        return Math.atan(yFromCenter/xFromCenter);
    }
<<<<<<< HEAD
}
=======
}
>>>>>>> 49c15ea78acebd0a44ca5f18d3195df95fa0ef71
