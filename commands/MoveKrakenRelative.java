// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KrakenSubsystem;

/**
 * Example command that moves the motor relative to its current position.
 * For example, if the motor is at 5 rotations and you call this with 2,
 * it will move to 7 rotations.
 */
public class MoveKrakenRelative extends Command {
  private final KrakenSubsystem krakenSubsystem;
  private final double relativeDistance;
  private double targetPosition;
  private final double tolerance;
  
  /**
   * Creates a new MoveKrakenRelative command.
   * 
   * @param subsystem The KrakenSubsystem to use
   * @param distance The relative distance to move (in rotations)
   * @param tolerance The tolerance in rotations
   */
  public MoveKrakenRelative(KrakenSubsystem subsystem, double distance, double tolerance) {
    this.krakenSubsystem = subsystem;
    this.relativeDistance = distance;
    this.tolerance = tolerance;
    addRequirements(subsystem);
  }
  
  /**
   * Creates a new MoveKrakenRelative command with default tolerance.
   * 
   * @param subsystem The KrakenSubsystem to use
   * @param distance The relative distance to move (in rotations)
   */
  public MoveKrakenRelative(KrakenSubsystem subsystem, double distance) {
    this(subsystem, distance, 0.1);
  }

  @Override
  public void initialize() {
    // Calculate target position based on current position
    targetPosition = krakenSubsystem.getPosition() + relativeDistance;
    krakenSubsystem.setPosition(targetPosition);
  }

  @Override
  public void execute() {
    // Position control is handled by the motor controller
  }

  @Override
  
  public void end(boolean interrupted) {
    // Motor will hold position
  }

  @Override
  public boolean isFinished() {
    // Command finishes when close enough to target
    double error = Math.abs(krakenSubsystem.getPosition() - targetPosition);
    return error < tolerance;
  }
}
