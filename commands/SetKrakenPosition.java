// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KrakenSubsystem;

public class SetKrakenPosition extends Command {
  private final KrakenSubsystem krakenSubsystem;
  private final double targetPosition;
  private final double tolerance;
  
  /**
   * Creates a new SetKrakenPosition command.
   * 
   * @param subsystem The KrakenSubsystem to use
   * @param position The target position in rotations
   * @param tolerance The tolerance in rotations (default: 0.1)
   */
  public SetKrakenPosition(KrakenSubsystem subsystem, double position, double tolerance) {
    this.krakenSubsystem = subsystem;
    this.targetPosition = position;
    this.tolerance = tolerance;
    addRequirements(subsystem);
  }
  
  /**
   * Creates a new SetKrakenPosition command with default tolerance of 0.1 rotations.
   * 
   * @param subsystem The KrakenSubsystem to use
   * @param position The target position in rotations
   */
  public SetKrakenPosition(KrakenSubsystem subsystem, double position) {
    this(subsystem, position, 0.1);
  }

  @Override
  public void initialize() {
    // Set the target position when the command starts
    krakenSubsystem.setPosition(targetPosition);
  }

  @Override
  public void execute() {
    // The motor controller handles the position control automatically
    // This method runs every 20ms, but we don't need to do anything here
  }

  @Override
  public void end(boolean interrupted) {
    // Optionally stop the motor when the command ends
    // Comment this out if you want the motor to hold position
    // krakenSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // Command finishes when the motor is within tolerance of the target
    double error = Math.abs(krakenSubsystem.getPosition() - targetPosition);
    return error < tolerance;
  }
}
