// package frc.robot.subsystems.vision;

// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.estimation.TargetModel;
// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Swerve;

// public class ApriltagVision extends SubsystemBase {
    
//     private PhotonCamera mFrontRightCam, mBackLeftCam;
//     private PhotonPoseEstimator mFrontRightEstimator, mBackLeftEstimator;
//     private AprilTagFieldLayout mFieldLayout  = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
//     private PhotonPipelineResult mFrontRightAprilTagResult, mBackLeftAprilTagResult;

//     public ApriltagVision(){
//         PhotonCamera.setVersionCheckEnabled(false);

//         mFrontRightCam = new PhotonCamera(Constants.Vision.aprilTagFrontRight.camera);
//         mBackLeftCam = new PhotonCamera(Constants.Vision.aprilTagBackLeft.camera);

//         mFrontRightEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mFrontRightCam, Constants.Vision.aprilTagFrontRight.robotToCamera);
//         mBackLeftEstimator = new PhotonPoseEstimator(mFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackLeftCam, Constants.Vision.aprilTagBackLeft.robotToCamera);

//         mFrontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//         mBackLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

//         mFrontRightEstimator.setTagModel(TargetModel.kAprilTag36h11);
//         mBackLeftEstimator.setTagModel(TargetModel.kAprilTag36h11);

//         mFrontRightEstimator.setRobotToCameraTransform(Constants.Vision.aprilTagFrontRight.robotToCamera);
//         mBackLeftEstimator.setRobotToCameraTransform(Constants.Vision.aprilTagBackLeft.robotToCamera);

        
//     }

//     @Override
//     public void periodic() {
//         if (this.mFrontRightCam != null){
//             mFrontRightAprilTagResult = mFrontRightCam.getLatestResult();
//             mFrontRightEstimator.update(mFrontRightAprilTagResult);

//             if (mFrontRightAprilTagResult.getMultiTagResult().estimatedPose.isPresent){

//                 if (mFrontRightEstimator.update().isPresent()){
//                     m.poseEstimator.addVisionMeasurement(new Pose2d(, Timer.getFPGATimestamp() - mFrontRightAprilTagResult.getLatencyMillis());
//                 }  
                

//             }

//         }

//         if (this.mBackLeftCam != null){
//             mBackLeftAprilTagResult = mBackLeftCam.getLatestResult();
//             mBackLeftEstimator.update(mBackLeftAprilTagResult);

//             if (mBackLeftAprilTagResult.getMultiTagResult().estimatedPose.isPresent){

//                 if (mBackLeftEstimator.update().isPresent()){
//                     Swerve.poseEstimator.addVisionMeasurement(mBackLeftEstimator.update().get().estimatedPose.toPose2d(), Timer.getFPGATimestamp() - mBackLeftAprilTagResult.getLatencyMillis());
//                 }
//             }

//         }

//     }
// }
