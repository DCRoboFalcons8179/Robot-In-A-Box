// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KrakenSubsystem;

public class RunKrakenAtSpeed extends Command {
  private final KrakenSubsystem krakenSubsystem;
  private final double targetSpeed;

  /**
   * Creates a new RunKrakenAtSpeed command. Runs the motor at a constant velocity (speed).
   *
   * @param subsystem The KrakenSubsystem to use
   * @param speed The target speed in arm rotations per second
   */
  public RunKrakenAtSpeed(KrakenSubsystem subsystem, double speed) {
    this.krakenSubsystem = subsystem;
    this.targetSpeed = speed;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // Set the target speed when the command starts
    krakenSubsystem.setSpeed(targetSpeed);
  }

  @Override
  public void execute() {
    // The motor controller handles the velocity control automatically
    // This runs continuously while the command is active
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the motor when the command ends
    krakenSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // This command runs indefinitely until interrupted
    // To make it stop automatically, return true here with a condition
    return false;
  }
}
