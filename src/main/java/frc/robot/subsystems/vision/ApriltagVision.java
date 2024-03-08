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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class ApriltagVision extends SubsystemBase {
    
    private PhotonCamera mFrontRightCam, mBackLeftCam;
    private PhotonPoseEstimator mFrontRightEstimator, mBackLeftEstimator;
    private AprilTagFieldLayout mFieldLayout  = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPipelineResult mFrontRightAprilTagResult, mBackLeftAprilTagResult;
    private Optional<EstimatedRobotPose> mFrontRight, mBackLeft;

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

        mFrontRightEstimator.setRobotToCameraTransform(Constants.Vision.aprilTagFrontRight.robotToCamera);
        mBackLeftEstimator.setRobotToCameraTransform(Constants.Vision.aprilTagBackLeft.robotToCamera);

        
    }

    @Override
    public void periodic() {
        if (Constants.Vision.visionTurnedOn){
            if (this.mFrontRightCam != null){
                mFrontRightAprilTagResult = mFrontRightCam.getLatestResult();

                mFrontRight = getBackLeftEstimatedGlobalPose(Swerve.poseEstimator.getEstimatedPosition(), mFrontRightAprilTagResult);

                if (mFrontRight.isPresent()){
                    Swerve.poseEstimator.addVisionMeasurement(new Pose2d(mFrontRight.get().estimatedPose.toPose2d().getTranslation(), Swerve.poseEstimator.getEstimatedPosition().getRotation()), mFrontRightAprilTagResult.getTimestampSeconds());
                }

            }

            if (this.mBackLeftCam != null){
                mBackLeftAprilTagResult = mBackLeftCam.getLatestResult();

                if (mBackLeftAprilTagResult.getTargets().size() >= 2){
                    mBackLeft = getFrontRightEstimatedGlobalPose(Swerve.poseEstimator.getEstimatedPosition(), mBackLeftAprilTagResult);

                    if (mBackLeft.isPresent()){
                        Swerve.poseEstimator.addVisionMeasurement(new Pose2d(mBackLeft.get().estimatedPose.toPose2d().getTranslation(), Swerve.poseEstimator.getEstimatedPosition().getRotation()), mBackLeftAprilTagResult.getTimestampSeconds());
                    }
                }


            }
        }


    }

    public Optional<EstimatedRobotPose> getFrontRightEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult mFrontRightAprilTagResult) {
        mFrontRightEstimator.setReferencePose(prevEstimatedRobotPose);
        return mFrontRightEstimator.update(mFrontRightAprilTagResult);
    }

    public Optional<EstimatedRobotPose> getBackLeftEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult mBackLeftAprilTagResult) {
        mBackLeftEstimator.setReferencePose(prevEstimatedRobotPose);
        return mBackLeftEstimator.update(mBackLeftAprilTagResult);
    }


    
}
