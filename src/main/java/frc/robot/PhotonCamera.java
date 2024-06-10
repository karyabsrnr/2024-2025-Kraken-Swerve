package frc.robot;

import java.util.List;

import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonCamera {
    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera();
    
    // Query the latest result from PhotonVision
    PhotonPipelineResult result = camera.getLatestResult();


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

public void captureImage() {
// Capture pre-process camera stream image
camera.takeInputSnapshot();
// Capture post-process camera stream image
camera.takeInputSnapshot();
}

public void driverMode() {
// Set driver mode to on.
camera.setDriverMode(true);
// Change pipeline to 2
camera.setPipelineIndex(2);
// Blink the LEDs.
camera.setLED(VisionLEDMode.kBlink);

}
// // Get the pipeline latency.
// private void takeInputSnapshot() {
//     // TODO Auto-generated method stub
//     throw new UnsupportedOperationException("Unimplemented method 'takeInputSnapshot'");
// }
double latencySeconds = result.getLatencyMillis() / 1000.0;

}
