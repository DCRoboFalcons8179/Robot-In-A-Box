// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.KrakenConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoMoveToAprilTagID;
import frc.robot.commands.MoveToPositionIfAprilTagSeen;
import frc.robot.commands.RunKrakenAtSpeed;
import frc.robot.commands.SetKrakenPosition;
import frc.robot.subsystems.KrakenSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  // Subsystems
  private final KrakenSubsystem krakenSubsystem = new KrakenSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Example button bindings - customize these to your preference

    // A button - go to home position (0 rotations)
    driverController
        .a()
        .onTrue(new SetKrakenPosition(krakenSubsystem, KrakenConstants.POSITION_HOME));

    // B button - go to mid position
    driverController
        .b()
        .onTrue(new SetKrakenPosition(krakenSubsystem, KrakenConstants.POSITION_MID));

    // Y button - go to high position
    driverController
        .y()
        .onTrue(new SetKrakenPosition(krakenSubsystem, KrakenConstants.POSITION_HIGH));

    // X button - reset encoder to 0
    driverController
        .x()
        .onTrue(Commands.runOnce(() -> krakenSubsystem.resetPosition(), krakenSubsystem));

    // D-pad Up - run at constant speed (0.5 rotations per second)
    driverController.povUp().whileTrue(new RunKrakenAtSpeed(krakenSubsystem, 5.0));

    // D-pad Down - move to position 12 if AprilTag ID 15 is seen
    driverController
        .povDown()
        .onTrue(new MoveToPositionIfAprilTagSeen(visionSubsystem, krakenSubsystem));
  }

  public Command getAutonomousCommand() {
    // Autonomous command - detect AprilTag ID (1-15) and move to that many rotations
    // ID 1 -> 1.0 rotations, ID 2 -> 2.0 rotations, etc.
    return new AutoMoveToAprilTagID(visionSubsystem, krakenSubsystem);
  }
}
