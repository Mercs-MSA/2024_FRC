package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

public class CustomGamePieceVision extends SubsystemBase{
    private NetworkTableInstance ntInst;
    private DoubleTopic yawTopic;
    private DoubleSubscriber yawSubscriber;

    double gamePieceYaw = 999.0;

    public CustomGamePieceVision(String yawString){
        ntInst = NetworkTableInstance.getDefault();

        // get a topic from a NetworkTableInstance
        // the topic name in this case is the full name
        this.yawTopic = ntInst.getDoubleTopic("/Vision/" + yawString);

        this.yawSubscriber = this.yawTopic.subscribe(999.0);
    }

    @Override
    public void periodic(){
        gamePieceYaw = yawSubscriber.get();

        // Update SmartDashboard for game piece
        SmartDashboard.putNumber("Game Piece Yaw", gamePieceYaw);
    }

    /**
     * Gets the Yaw angle of the game piece.
     * @return The Yaw angle.
     */
    public double getGamePieceYaw(){
        return gamePieceYaw;
    }

    /**
     * Align center of camera with game piece
     * @param Swerve drive subsystem
     */
    public void alignNoteYaw(Swerve swerve) {
        swerve.drive(new Translation2d(0, 0), gamePieceYaw/20, false, true);
    }

     /**
     * Generate yaw command based on Note position in camera view
     * Output is a size 2 double array with the angle command 1st and the forward translation 2nd
     */
    public double[] alignNoteCommands() {
        double[] yaw_and_translate = {gamePieceYaw/20, 0.1};
        return yaw_and_translate;
    }

}
