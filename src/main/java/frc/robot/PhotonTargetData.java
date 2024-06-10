package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonTargetData {
    // Check if the latest result has any targets.
    private PhotonCamera camera;
    PhotonPipelineResult result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

// Get a list of currently tracked targets.
List<PhotonTrackedTarget> targets = result.getTargets();

// Get the current best target.
PhotonTrackedTarget target = result.getBestTarget();

// Get information from target.
double yaw = target.getYaw();
double pitch = target.getPitch();
double area = target.getArea();
double skew = target.getSkew();
Transform2d pose = target.getCameraToTarget();
List<TargetCorner> corners = target.getCorners();

// Get information from target.
int targetID = target.getFiducialId();
double poseAmbiguity = target.getPoseAmbiguity();
Transform3d bestCameraToTarget = target.getBestCameraToTarget();
Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

// Capture pre-process camera stream image
amera.takeInputSnapshot();

// Capture post-process camera stream image
camera.takeOutputSnapshot();
}
