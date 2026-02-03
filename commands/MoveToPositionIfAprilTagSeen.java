// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.KrakenSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MoveToPositionIfAprilTagSeen extends Command {
  private final VisionSubsystem visionSubsystem;
  private final KrakenSubsystem krakenSubsystem;
  private final int targetAprilTagId;
  private final double targetPosition;
  private boolean tagWasSeen = false;
  
  /**
   * Creates a command that moves to a position if a specific AprilTag is visible.
   * 
   * @param vision The VisionSubsystem
   * @param kraken The KrakenSubsystem
   * @param aprilTagId The AprilTag ID to look for
   * @param position The position to move to (in mechanism rotations)
   */
  public MoveToPositionIfAprilTagSeen(VisionSubsystem vision, KrakenSubsystem kraken, 
                                       int aprilTagId, double position) {
    this.visionSubsystem = vision;
    this.krakenSubsystem = kraken;
    this.targetAprilTagId = aprilTagId;
    this.targetPosition = position;
    
    // Only require vision subsystem - kraken will be commanded separately
    addRequirements(vision);
  }
  
  /**
   * Convenience constructor using constants from VisionConstants
   */
  public MoveToPositionIfAprilTagSeen(VisionSubsystem vision, KrakenSubsystem kraken) {
    this(vision, kraken, VisionConstants.TARGET_APRILTAG_ID, VisionConstants.APRILTAG_TARGET_POSITION);
  }

  @Override
  public void initialize() {
    tagWasSeen = false;
  }

  @Override
  public void execute() {
    // Check if we can see the target AprilTag
    if (visionSubsystem.canSeeAprilTag(targetAprilTagId)) {
      // Tag is visible! Command the motor to move to target position
      if (!tagWasSeen) {
        // Only command once when first detected
        krakenSubsystem.setPosition(targetPosition);
        tagWasSeen = true;
        System.out.println("AprilTag " + targetAprilTagId + " detected! Moving to position " + targetPosition);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("Command interrupted");
    } else if (!tagWasSeen) {
      System.out.println("AprilTag " + targetAprilTagId + " not found");
    }
  }

  @Override
  public boolean isFinished() {
    // Command finishes after checking and commanding the motor (or timing out)
    // The motor will continue to the position even after this command ends
    return tagWasSeen;
  }
}
