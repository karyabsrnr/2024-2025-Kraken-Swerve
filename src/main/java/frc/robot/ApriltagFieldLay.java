package frc.robot;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ApriltagFieldLay {
    // The field from AprilTagFields will be different depending on the game.
AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); {

//Forward Camera
PhotonCamera cam = new PhotonCamera();
//Cam mounted facing forward, half a meter forward of center, half a meter up from center.
Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), 
new Rotation3d(0,0,0));

// Construct PhotonPoseEstimator
PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, 
PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);
}
}