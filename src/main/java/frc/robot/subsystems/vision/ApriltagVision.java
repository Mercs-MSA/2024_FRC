package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ApriltagVision extends SubsystemBase {
    
    private PhotonCamera mFrontRightCam, mBackLeftCam = new PhotonCamera(null);
    private PhotonPoseEstimator mFrontRightEstimator, mBackLeftEstimator = new PhotonPoseEstimator(null, null, null);
    private AprilTagFieldLayout mFieldLayout  = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public ApriltagVision(){
        mFrontRightCam = new PhotonCamera(Constants.Vision.aprilTagFrontRight.camera);
        mBackLeftCam = new PhotonCamera(Constants.Vision.aprilTagBackLeft.camera);

        mFrontRightCam.setVersionCheckEnabled(false);
        mBackLeftCam.setVersionCheckEnabled(false);

        mFrontRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontRightCam, Constants.Vision.aprilTagFrontRight.robotToCamera);
        mBackLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackLeftCam, Constants.Vision.aprilTagBackLeft.robotToCamera);

        mFrontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mBackLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);


        if(mFrontRightCam != null){
            if (mFrontRightEstimator.update().isPresent()){
                RobotContainer.s_Swerve.poseEstimator.addVisionMeasurement(mFrontRightEstimator.update().get().estimatedPose.toPose2d(), mFrontRightEstimator.update().get().timestampSeconds);
            }
        }
        if(mBackLeftCam != null){
            if (mBackLeftEstimator.update().isPresent()){
                RobotContainer.s_Swerve.poseEstimator.addVisionMeasurement(mBackLeftEstimator.update().get().estimatedPose.toPose2d(), mBackLeftEstimator.update().get().timestampSeconds);
            }
        }

    }

    @Override
    public void simulationPeriodic() {
    }

}
