// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KrakenSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Autonomous command that continuously moves the Kraken to a position based on the AprilTag ID
 * seen. If AprilTag ID 1 is seen, move to 1.0 rotations. If AprilTag ID 2 is seen, move to 2.0
 * rotations. And so on up to ID 15 -> 15.0 rotations. This command continuously monitors for new
 * tags and will move to new positions as different tags are detected.
 */
public class AutoMoveToAprilTagID extends Command {
  private final VisionSubsystem visionSubsystem;
  private final KrakenSubsystem krakenSubsystem;
  private int lastDetectedTagId = -1;

  public AutoMoveToAprilTagID(VisionSubsystem visionSubsystem, KrakenSubsystem krakenSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.krakenSubsystem = krakenSubsystem;
    addRequirements(visionSubsystem, krakenSubsystem);
  }

  @Override
  public void initialize() {
    lastDetectedTagId = -1;
    System.out.println("AutoMoveToAprilTagID: Starting continuous AprilTag monitoring...");
  }

  @Override
  public void execute() {
    // Continuously check for AprilTags
    if (visionSubsystem.hasTargets()) {
      var bestTarget = visionSubsystem.getBestTarget();
      if (bestTarget != null) {
        int aprilTagId = bestTarget.getFiducialId();

        // Check if this is a NEW AprilTag (different from the last one we saw)
        if (aprilTagId != lastDetectedTagId) {
          // Check if the AprilTag ID is in the valid range (1-32)
          if (aprilTagId >= 1 && aprilTagId <= 32) {
            double targetPosition = aprilTagId; // ID 1 -> 1.0 rotations, ID 2 -> 2.0, etc.
            System.out.println(
                "AutoMoveToAprilTagID: Detected NEW AprilTag ID "
                    + aprilTagId
                    + ", moving to "
                    + targetPosition
                    + " rotations");
            krakenSubsystem.setPosition(targetPosition);
            lastDetectedTagId = aprilTagId;
          } else {
            System.out.println(
                "AutoMoveToAprilTagID: Detected AprilTag ID "
                    + aprilTagId
                    + " (out of range 1-32), ignoring");
          }
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("AutoMoveToAprilTagID: Command interrupted");
    } else {
      System.out.println("AutoMoveToAprilTagID: Command finished");
    }
    krakenSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // This command runs for the entire autonomous period (never finishes on its own)
    // It will be interrupted when autonomous ends
    return false;
  }
}
