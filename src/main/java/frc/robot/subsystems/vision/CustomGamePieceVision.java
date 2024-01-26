package frc.robot.subsystems.vision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.NullNode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

public class CustomGamePieceVision extends SubsystemBase{
    private NetworkTableInstance ntInst;
    private StringTopic pipelineTopic;
    private StringSubscriber pipelineSubscriber;

    private JsonNode objectPipelineResult;
    private JsonNode bestGamePiece;

    double gamePieceYaw, gamePieceArea, gamePiecePerimeter = 999.0;

    public CustomGamePieceVision(String topicString){
        ntInst = NetworkTableInstance.getDefault();

        // get a topic from a NetworkTableInstance
        // the topic name in this case is the full name
        this.pipelineTopic = ntInst.getStringTopic("/Vision/" + topicString);

        this.pipelineSubscriber = this.pipelineTopic.subscribe("{}");
    }

    @Override
    public void periodic(){
        ObjectMapper mapper = new ObjectMapper();
        try {
            objectPipelineResult = mapper.readTree(pipelineSubscriber.get().toString());
        } catch (JsonProcessingException e) {
            gamePieceYaw = 999.0;
            gamePieceArea = 999.0;
            gamePiecePerimeter = 999.0;
            return;
        }

        bestGamePiece = objectPipelineResult.get("best"); // get best object

        if (bestGamePiece == NullNode.instance || bestGamePiece == null) {
            // there are no notes
            gamePieceYaw = 999.0;
            gamePieceArea = 999.0;
            gamePiecePerimeter = 999.0;
        } else {
            gamePieceYaw = bestGamePiece.get("yaw").asDouble();
            gamePieceArea = bestGamePiece.get("area").asDouble();
            gamePiecePerimeter = bestGamePiece.get("perimeter").asDouble();
        }

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
     * Gets the Area (px) of the game piece.
     * @return The Area in px.
     */
    public double getGamePieceArea(){
        return gamePieceArea;
    }

    /**
     * Align center of camera with game piece
     * @param Swerve drive subsystem
     */
    public void alignNoteYaw(Swerve swerve) {
        System.out.println("help me");
        swerve.drive(new Translation2d(0, 0), gamePieceYaw/20, false, true);
    }

}
