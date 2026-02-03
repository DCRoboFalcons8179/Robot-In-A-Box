// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;
  
  // Elastic NetworkTables publishers for vision telemetry
  private final BooleanPublisher hasTargetPub;
  private final IntegerPublisher targetIdPub;
  private final DoublePublisher targetYawPub;
  private final DoublePublisher targetPitchPub;
  private final DoublePublisher targetAreaPub;
  
  public VisionSubsystem() {
    // Initialize PhotonVision camera
    camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
    
    // Initialize Elastic NetworkTables publishers
    var table = NetworkTableInstance.getDefault().getTable("Vision");
    hasTargetPub = table.getBooleanTopic("Has Target").publish();
    targetIdPub = table.getIntegerTopic("AprilTag ID").publish();
    targetYawPub = table.getDoubleTopic("Target Yaw").publish();
    targetPitchPub = table.getDoubleTopic("Target Pitch").publish();
    targetAreaPub = table.getDoubleTopic("Target Area").publish();
  }

  /**
   * Get the latest result from the camera
   * @return The latest PhotonPipelineResult
   */
  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }
  
  /**
   * Check if the camera has any targets
   * @return true if targets are detected
   */
  public boolean hasTargets() {
    return camera.getLatestResult().hasTargets();
  }
  
  /**
   * Get the best target from the camera
   * @return The best PhotonTrackedTarget, or null if no targets
   */
  public PhotonTrackedTarget getBestTarget() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      return result.getBestTarget();
    }
    return null;
  }
  
  /**
   * Check if a specific AprilTag ID is visible
   * @param id The AprilTag ID to check for
   * @return true if the tag is visible
   */
  public boolean canSeeAprilTag(int id) {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.getFiducialId() == id) {
          return true;
        }
      }
    }
    return false;
  }
  
  /**
   * Get a specific AprilTag by ID
   * @param id The AprilTag ID to get
   * @return The PhotonTrackedTarget for that ID, or null if not found
   */
  public PhotonTrackedTarget getAprilTag(int id) {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.getFiducialId() == id) {
          return target;
        }
      }
    }
    return null;
  }

  @Override
  public void periodic() {
    // Update telemetry
    var result = camera.getLatestResult();
    hasTargetPub.set(result.hasTargets());
    
    if (result.hasTargets()) {
      var bestTarget = result.getBestTarget();
      targetIdPub.set(bestTarget.getFiducialId());
      targetYawPub.set(bestTarget.getYaw());
      targetPitchPub.set(bestTarget.getPitch());
      targetAreaPub.set(bestTarget.getArea());
    } else {
      targetIdPub.set(-1);
      targetYawPub.set(0);
      targetPitchPub.set(0);
      targetAreaPub.set(0);
    }
  }
}
