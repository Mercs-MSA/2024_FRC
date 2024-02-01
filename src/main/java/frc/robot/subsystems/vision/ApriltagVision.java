package frc.robot.subsystems.vision;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ApriltagVision extends SubsystemBase {

    private String cameraName;
    private PhotonCamera camera;
    private PhotonPipelineResult aprilTagResult;
    private boolean aprilTagHasTargets;
    private List<PhotonTrackedTarget> aprilTagTargets;
    private PhotonTrackedTarget aprilTagBestTarget;
    private AprilTagFieldLayout aprilTagFieldLayout;
    // private PhotonPoseEstimator poseEstimator;
    private int fiducialID;
    private Transform3d robotToCam;
    private double aprilTagX, aprilTagY, aprilTagZAngle, aprilTagZ = -1;
    private Pose2d globalPoseEstimate = new Pose2d();
    private Transform3d fieldToCamera;
    // private Field2d apriltaField2d = new Field2d();

    public ApriltagVision(String cameraName) {
        this.cameraName = cameraName;
        camera = new PhotonCamera(cameraName);
        aprilTagResult = new PhotonPipelineResult();
        aprilTagHasTargets = false;
        // aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2024Crescendo.m_resourceFile);
        // this.robotToCam = robotToCam; 
        // poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
    }

    public void periodic(){
        aprilTagResult = camera.getLatestResult();
        
        aprilTagHasTargets = aprilTagResult.hasTargets();

        if (aprilTagHasTargets) {
            aprilTagTargets = aprilTagResult.getTargets();
            aprilTagBestTarget = aprilTagResult.getBestTarget();

            fiducialID = aprilTagBestTarget.getFiducialId();
            aprilTagX = aprilTagBestTarget.getBestCameraToTarget().getX();
            aprilTagY = aprilTagBestTarget.getBestCameraToTarget().getY();
            aprilTagZ = aprilTagBestTarget.getBestCameraToTarget().getZ();
            aprilTagZAngle = aprilTagBestTarget.getBestCameraToTarget().getRotation().getAngle();
            if (aprilTagResult.getMultiTagResult().estimatedPose.isPresent){
                fieldToCamera = aprilTagResult.getMultiTagResult().estimatedPose.best;
            }   
        } 
        else {
            fiducialID = -1;
            aprilTagX = -1.0;
            aprilTagY = -1.0;
            aprilTagZ = -1.0;
            aprilTagZAngle = -1.0;
        }

        if (aprilTagHasTargets){
            // Update SmartDashboard for AprilTag
            SmartDashboard.putNumber(cameraName + " Fiducial ID", fiducialID);
            SmartDashboard.putNumber(cameraName + " AprilTag X (m)", aprilTagX);
            SmartDashboard.putNumber(cameraName + " AprilTag Y (m)", aprilTagY);
            SmartDashboard.putNumber(cameraName + " AprilTag Z (m)", aprilTagZ);
            SmartDashboard.putNumber(cameraName + " AprilTag Z Angle", aprilTagZAngle);
            if (aprilTagResult.getMultiTagResult().estimatedPose.isPresent){
                SmartDashboard.putNumber(cameraName + " Field To Camera Pose Estimate X", fieldToCamera.getX());
                SmartDashboard.putNumber(cameraName + " Field To Camera Pose Estimate Y", fieldToCamera.getY());
                SmartDashboard.putNumber(cameraName + " Field To Camera Pose Estimate Z", fieldToCamera.getZ());
                SmartDashboard.putNumber(cameraName + " Field To Camera Pose Estimate Angle", fieldToCamera.getRotation().getAngle());
            }
            globalPoseEstimate = new Pose2d(fieldToCamera.getX(), fieldToCamera.getY(), new Rotation2d(fieldToCamera.getRotation().getX(), fieldToCamera.getRotation().getY()));
            // apriltaField2d.setRobotPose(globalPoseEstimate);
        }

        // SmartDashboard.putData("estimated pose", apriltaField2d);



    }

    // public void updateEstimatedGlobalPose() {
    //     poseEstimator.update();
    //     globalPoseEstimate = poseEstimator.update().get().estimatedPose;
    // }

    public double getTimestampSeconds(){
        return this.aprilTagResult.getTimestampSeconds();
    }

    public boolean hasTargets(){
        return this.aprilTagHasTargets;
    }

    public boolean hasMultiTagEstimatedPose(){
        return this.aprilTagResult.getMultiTagResult().estimatedPose.isPresent;
    }

    public Pose2d getGlobalPoseEstimate() {
        return this.globalPoseEstimate;
    }

    /**
     * Gets the Fiducial ID of the AprilTag.
     * @return The Fiducial ID.
     */
    public int getFiducialID(){
        return fiducialID;
    }

    /**
     * Gets the X coordinate of the AprilTag in meters.
     * @return The X coordinate.
     */
    public double getAprilTagX(){
        return aprilTagX;
    }

    /**
     * Gets the Y coordinate of the AprilTag in meters.
     * @return The Y coordinate.
     */
    public double getAprilTagY(){
        return aprilTagY;
    }

    /**
     * Gets the Z coordinate of the AprilTag in meters.
     * @return The Z coordinate.
     */
    public double getAprilTagZ(){
        return aprilTagZ;
    }

    /**
     * Gets the Z angle of the AprilTag in degrees.
     * @return The Z angle.
     */
    public double getAprilTagZAngle(){
        return aprilTagZAngle * (180 / Math.PI);
    }
}
