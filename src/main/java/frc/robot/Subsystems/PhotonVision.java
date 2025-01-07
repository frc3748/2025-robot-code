package frc.robot.Subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Drivetrain.Drive;

public class PhotonVision implements Sendable {
    static PhotonCamera camera;
    static public EstimatedRobotPose pose;
    static Transform3d robotToCam;
    static Transform3d cameraToRobot;
    static AprilTagFieldLayout aprilTagFieldLayout;
    static PhotonPoseEstimator photonPoseEstimator;
    public static Notifier notifier;
    static Drive drive;

    static {
        PortForwarder.add(5800, "3748photonvision", 5800);
        camera = new PhotonCamera("photonvision");
        // get result using camera.getLatestResult()

        //Cam mounted facing forward, -0.381 meter forward of center, 0.558 a meter up from center.
        robotToCam = new Transform3d(new Translation3d(-0.332613, 0.0, -0.1529588), new Rotation3d(Math.PI, -Math.PI/4, Math.PI)); 
        
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }

        PhotonVision.notifier = new Notifier(PhotonVision::getPose);
        PhotonVision.notifier.startPeriodic(.02);
    }

    // get field oriented robot pose w/ multitag processing
    public static EstimatedRobotPose getPose(){ 
        SmartDashboard.putNumber("VisionUpdateTracker", Timer.getFPGATimestamp());
        if (photonPoseEstimator == null){
            return null;
        }
        EstimatedRobotPose currentPose = photonPoseEstimator.update().orElse(null);
        if(currentPose == null){
            return pose;
        }
        if (pose == null){
            pose = currentPose;
            return pose;
        } 
        if (pose.timestampSeconds == currentPose.timestampSeconds) {
            return pose;
        }
        pose = currentPose;
        return pose;
    }

    public static void getDrive(Drive drivevar){
        PhotonVision.drive = drivevar;
    }

    // get distance from closest apriltag
    public static Double getDistanceFromClosestTag(){
        double distance;
        double min = 100000;
        List<PhotonTrackedTarget> targets;
        if (pose != null){
            targets = pose.targetsUsed;
        } else {
            return null;
        }
        for (PhotonTrackedTarget target : targets){
            distance = Math.abs(target.getBestCameraToTarget().getTranslation().getNorm());
            if (distance < min) min = distance;
        }
            return Double.valueOf(min);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        Pose2d pose = getPose().estimatedPose.toPose2d();
        if(pose != null){
            builder.addDoubleProperty("rpx", () -> pose.getX(), null);
            builder.addDoubleProperty("rpy", () -> pose.getY(), null);
            builder.addDoubleProperty("rpangle", () -> pose.getRotation().getDegrees(), null);
        }
        else {
            builder.addDoubleProperty("rpx", () -> Double.NaN, null);
            builder.addDoubleProperty("rpy", () -> Double.NaN, null);
            builder.addDoubleProperty("rpangle", () -> Double.NaN, null);
        }
    }       
}