package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
//THIS IS BAD :(
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import static frc.robot.Constants.PhotonVision.*;



public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private List<PhotonPipelineResult> unreadResults = new ArrayList<>();
    private PhotonPipelineResult latestResult = null;
    private double lastEstTimestamp = 0;
    private double lastYaw = 0;
    private double targetRange;
    
    public VisionSubsystem(String cameraName) {
        camera = new PhotonCamera(cameraName);
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        Transform3d robotToCam;
        if (cameraName == "Cam 1") {
            // Cam 1 to robot
            robotToCam = new Transform3d(new Translation3d(0.381, 0.0, -0.10), new Rotation3d(0,0,0));
        } else {
            // Cam 2 to robot
            robotToCam = new Transform3d(new Translation3d(-0.356, 0.0, 0.05), new Rotation3d(0,0,180));
        }
        photonEstimator =
                new PhotonPoseEstimator(
                        tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }
    
    public void getLatestResult() {
        this.unreadResults = camera.getAllUnreadResults();
       
        if (!unreadResults.isEmpty()) {
            latestResult = unreadResults.get(unreadResults.size() - 1);
            SmartDashboard.putBoolean("PhotonVision: Has Target", true);
            SmartDashboard.putNumber("PhotonVision: Targets Count", unreadResults.size());
        }
        else {
            latestResult = null;
            SmartDashboard.putBoolean("PhotonVision: Has Target", false);
            System.out.println("[PhotonVision] No new targets detected.");
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        getLatestResult();
        var visionEst = photonEstimator.update(latestResult);
        double latestTimestamp = latestResult.getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }


    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = singleTagStdDevs;
        List<PhotonTrackedTarget> targets = latestResult.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = multiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));


        return estStdDevs;
    }

    public double getYaw() {
        getLatestResult();
        if (latestResult != null && latestResult.hasTargets()) {
            var target = latestResult.getBestTarget();
            lastYaw = target.getYaw();

            return target.getYaw();
        }
        if (Math.abs(lastYaw) < 5) {
            return 0;
        } else {
            return lastYaw;
        }
    }

    public double getDistance() {
        getLatestResult();
        if (latestResult != null && latestResult.hasTargets()) {
            var target = latestResult.getBestTarget();
            targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        0.5, // Measured with a tape measure, or in CAD.
                                        1.435, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));
            return targetRange;
        }
        return 0.0;
    }

}