// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenConstants;

public class KrakenSubsystem extends SubsystemBase {
  // Create the Kraken X60 motor (TalonFX controller)
  private final TalonFX krakenMotor;
  
  // Motion Magic control request for smooth, speed-limited movement
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  
  // Velocity control request for constant speed (uses Slot 1)
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(1);
  
  // Track target position for logging
  private double targetPosition = 0.0;
  
  // Elastic NetworkTables publishers for efficient telemetry
  private final DoublePublisher actualPositionPub;
  private final DoublePublisher targetPositionPub;
  private final DoublePublisher positionErrorPub;
  private final DoublePublisher motorPositionPub;  // Added: Raw motor position
  private final DoublePublisher velocityPub;
  private final DoublePublisher motorVoltagePub;
  private final DoublePublisher supplyCurrentPub;
  private final DoublePublisher statorCurrentPub;
  private final DoublePublisher temperaturePub;
  private final BooleanPublisher atTargetPub;
  
  public KrakenSubsystem() {
    // Initialize the motor using the new Phoenix 6 API
    krakenMotor = new TalonFX(KrakenConstants.MOTOR_CAN_ID);
    
    // Initialize Elastic NetworkTables publishers
    var table = NetworkTableInstance.getDefault().getTable("Kraken");
    actualPositionPub = table.getDoubleTopic("Actual Position (Mechanism)").publish();
    targetPositionPub = table.getDoubleTopic("Target Position (Mechanism)").publish();
    positionErrorPub = table.getDoubleTopic("Position Error (Mechanism)").publish();
    motorPositionPub = table.getDoubleTopic("Motor Position (Raw)").publish();
    velocityPub = table.getDoubleTopic("Velocity (Mechanism)").publish();
    motorVoltagePub = table.getDoubleTopic("Motor Voltage").publish();
    supplyCurrentPub = table.getDoubleTopic("Supply Current").publish();
    statorCurrentPub = table.getDoubleTopic("Stator Current").publish();
    temperaturePub = table.getDoubleTopic("Temperature (C)").publish();
    atTargetPub = table.getBooleanTopic("At Target").publish();
    
    // Configure the motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Slot 0: PID values for Position Control
    config.Slot0.kP = KrakenConstants.kP;
    config.Slot0.kI = KrakenConstants.kI;
    config.Slot0.kD = KrakenConstants.kD;
    config.Slot0.kS = KrakenConstants.kS;
    config.Slot0.kV = KrakenConstants.kV;
    config.Slot0.kA = KrakenConstants.kA;
    
    // Slot 1: PID values for Velocity Control
    config.Slot1.kP = KrakenConstants.kP_Velocity;
    config.Slot1.kI = KrakenConstants.kI_Velocity;
    config.Slot1.kD = KrakenConstants.kD_Velocity;
    config.Slot1.kS = KrakenConstants.kS;
    config.Slot1.kV = KrakenConstants.kV;
    config.Slot1.kA = KrakenConstants.kA;
    
    // Set motor to brake mode (or NeutralModeValue.Coast for coast mode)
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    // Set motion magic constraints to control speed and acceleration
    config.MotionMagic.MotionMagicCruiseVelocity = KrakenConstants.MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = KrakenConstants.MAX_ACCELERATION;
    config.MotionMagic.MotionMagicJerk = KrakenConstants.MAX_JERK;
    
    // Optional: Set current limits to protect the motor
    config.CurrentLimits.SupplyCurrentLimit = KrakenConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = KrakenConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Apply the configuration
    krakenMotor.getConfigurator().apply(config);
    
    // Reset the encoder position to 0
    krakenMotor.setPosition(0);
  }

  /**
   * Set the mechanism to a target position (in mechanism rotations)
   * Automatically converts to motor rotations based on gear ratio
   * Uses Motion Magic for smooth, speed-limited movement
   * @param mechanismRotations The target position in mechanism rotations
   */
  public void setPosition(double mechanismRotations) {
    targetPosition = mechanismRotations;
    // Convert mechanism rotations to motor rotations
    double motorRotations = mechanismRotations / KrakenConstants.GEAR_RATIO;
    krakenMotor.setControl(motionMagicRequest.withPosition(motorRotations));
  }
  
  /**
   * Set the mechanism to run at a constant speed (velocity control)
   * Automatically converts to motor velocity based on gear ratio
   * Uses acceleration limiting and FOC for smooth operation
   * @param mechanismRotationsPerSecond The target speed in mechanism rotations per second
   */
  public void setSpeed(double mechanismRotationsPerSecond) {
    // Convert mechanism velocity to motor velocity
    double motorRotationsPerSecond = mechanismRotationsPerSecond / KrakenConstants.GEAR_RATIO;
    krakenMotor.setControl(velocityRequest
        .withVelocity(motorRotationsPerSecond)
        .withAcceleration(40.0 / KrakenConstants.GEAR_RATIO)  // Add acceleration limiting
        .withEnableFOC(true));  // Enable Field Oriented Control for smoother operation
  }
  
  /**
   * Get the target position the mechanism is trying to reach
   * @return The target position in mechanism rotations
   */
  public double getTargetPosition() {
    return targetPosition;
  }
  
  /**
   * Get the position error (difference between target and actual)
   * @return The position error in mechanism rotations
   */
  public double getPositionError() {
    return targetPosition - getPosition();
  }
  
  /**
   * Check if the motor is at the target position (within tolerance)
   * @return true if within 0.1 rotations of target
   */
  public boolean isAtTarget() {
    return Math.abs(getPositionError()) < 0.1;
  }
  
  /**
   * Get the motor's output voltage
   * @return The motor voltage in volts
   */
  public double getMotorVoltage() {
    return krakenMotor.getMotorVoltage().getValueAsDouble();
  }
  
  /**
   * Get the motor's supply current (battery current)
   * @return The supply current in amps
   */
  public double getSupplyCurrent() {
    return krakenMotor.getSupplyCurrent().getValueAsDouble();
  }
  
  /**
   * Get the motor's stator current (actual motor current)
   * @return The stator current in amps
   */
  public double getStatorCurrent() {
    return krakenMotor.getStatorCurrent().getValueAsDouble();
  }
  
  /**
   * Get the motor's temperature
   * @return The motor temperature in Celsius
   */
  public double getTemperature() {
    return krakenMotor.getDeviceTemp().getValueAsDouble();
  }
  
  /**
   * Get the current position of the mechanism (in mechanism rotations)
   * @return The current position in mechanism rotations
   */
  public double getPosition() {
    // Convert motor rotations to mechanism rotations using gear ratio
    return krakenMotor.getPosition().getValueAsDouble() * KrakenConstants.GEAR_RATIO;
  }
  
  /**
   * Get the current motor position (in motor rotations) - for debugging
   * @return The current motor position in rotations
   */
  public double getMotorPosition() {
    return krakenMotor.getPosition().getValueAsDouble();
  }
  
  /**
   * Reset the encoder position to 0
   */
  public void resetPosition() {
    krakenMotor.setPosition(0);
  }
  
  /**
   * Get the current velocity of the mechanism (in mechanism rotations per second)
   * @return The current velocity in mechanism rotations per second
   */
  public double getVelocity() {
    // Convert motor velocity to mechanism velocity using gear ratio
    return krakenMotor.getVelocity().getValueAsDouble() * KrakenConstants.GEAR_RATIO;
  }
  
  /**
   * Stop the motor
   */
  public void stop() {
    krakenMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Publish comprehensive telemetry to Elastic dashboard
    
    // Position data (in mechanism rotations)
    actualPositionPub.set(getPosition());
    targetPositionPub.set(getTargetPosition());
    positionErrorPub.set(getPositionError());
    
    // Raw motor position for debugging
    motorPositionPub.set(getMotorPosition());
    
    // Velocity data (in mechanism rotations per second)
    velocityPub.set(getVelocity());
    
    // Electrical data
    motorVoltagePub.set(getMotorVoltage());
    supplyCurrentPub.set(getSupplyCurrent());
    statorCurrentPub.set(getStatorCurrent());
    
    // Motor health
    temperaturePub.set(getTemperature());
    
    // Status indicators
    atTargetPub.set(Math.abs(getPositionError()) < 0.1);
  }
}
