package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class ApriltagVision extends SubsystemBase {
    
    private PhotonCamera mFrontRightCam, mBackLeftCam;
    private PhotonPoseEstimator mFrontRightEstimator, mBackLeftEstimator;
    private AprilTagFieldLayout mFieldLayout  = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPipelineResult mFrontRightAprilTagResult, mBackLeftAprilTagResult;

    public ApriltagVision(){
        PhotonCamera.setVersionCheckEnabled(false);

        mFrontRightCam = new PhotonCamera(Constants.Vision.aprilTagFrontRight.camera);
        mBackLeftCam = new PhotonCamera(Constants.Vision.aprilTagBackLeft.camera);

        mFrontRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontRightCam, Constants.Vision.aprilTagFrontRight.robotToCamera);
        mBackLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackLeftCam, Constants.Vision.aprilTagBackLeft.robotToCamera);

        mFrontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mBackLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        mFrontRightEstimator.setTagModel(TargetModel.kAprilTag36h11);
        mBackLeftEstimator.setTagModel(TargetModel.kAprilTag36h11);

        
    }

    @Override
    public void periodic() {
        if (this.mFrontRightCam != null){
            mFrontRightAprilTagResult = mFrontRightCam.getLatestResult();

            if (mFrontRightEstimator.update(mFrontRightAprilTagResult).isPresent()){
                Swerve.poseEstimator.addVisionMeasurement(mFrontRightEstimator.update().get().estimatedPose.toPose2d(), mFrontRightEstimator.update().get().timestampSeconds - mFrontRightAprilTagResult.getLatencyMillis());
            }
        }

        if (this.mBackLeftCam != null){
            mBackLeftAprilTagResult = mBackLeftCam.getLatestResult();

            if (mBackLeftEstimator.update(mBackLeftAprilTagResult).isPresent()){
                Swerve.poseEstimator.addVisionMeasurement(mBackLeftEstimator.update().get().estimatedPose.toPose2d(), mBackLeftEstimator.update().get().timestampSeconds - mBackLeftAprilTagResult.getLatencyMillis());
            }

        }

    }
}
