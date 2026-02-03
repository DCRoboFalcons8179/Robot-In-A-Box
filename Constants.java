// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {
  
  public static class KrakenConstants {
    // Motor CAN ID - CHANGE THIS to match your motor's ID
    public static final int MOTOR_CAN_ID = 1;
    
    // CANivore name - leave empty for default CAN bus, or specify your CANivore name like "canivore1"
    public static final String CAN_BUS = "";
    
    // Compound Gear Ratio Calculation:
    // 1. Kraken → 3:1 gearbox = 1/3 = 0.3333
    // 2. 22 tooth → 60 tooth belt = 22/60 = 0.3667
    // Total: (1/3) × (22/60) = 22/180 = 0.1222
    // This means: 1 motor rotation = 0.1222 arm rotations
    // Or: 8.18 motor rotations = 1 arm rotation
    public static final double GEAR_RATIO = (1.0 / 3.0) * (22.0 / 60.0);  // = 0.1222 (arm rotations per motor rotation)
    
    // PID Constants - TUNE THESE for your mechanism
    // Slot 0 - Position Control
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    // Slot 1 - Velocity Control
    public static final double kP_Velocity = 0.4;   // Lower P for velocity
    public static final double kI_Velocity = 0.0;
    public static final double kD_Velocity = 0.0;
    
    // Feedforward Constants
    public static final double kS = 0.0;  // Static friction voltage
    public static final double kV = 0.12;  // Velocity feedforward (12V per 100 rps = 0.12V per rps)
    public static final double kA = 0.0;  // Acceleration feedforward
    
    // Current Limits
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;  // Amps
    public static final double STATOR_CURRENT_LIMIT = 80.0;  // Amps
    
    // Motion Constraints - ADJUST THESE to control speed
    // Motion Magic will automatically:
    // 1. Accelerate quickly at the start (up to MAX_ACCELERATION)
    // 2. Cruise at high speed in the middle (MAX_VELOCITY)
    // 3. Decelerate smoothly as it approaches target
    public static final double MAX_VELOCITY = 60.0;        // Cruise speed (motor rotations/second) - increased for faster movement
    public static final double MAX_ACCELERATION = 120.0;   // How quickly it speeds up/slows down (motor rot/sec²) - increased for snappier response
    public static final double MAX_JERK = 2400.0;          // Smoothness of acceleration changes (motor rot/sec³)
    
    // Position tolerance for commands
    public static final double POSITION_TOLERANCE = 0.1;  // Rotations (mechanism rotations)
    
    // Example preset positions (in MECHANISM rotations, NOT motor rotations)
    // With 22:60 gear ratio, these are mechanism rotations
    public static final double POSITION_HOME = 0.0;   // Home position
    public static final double POSITION_MID = 5.0;    // 5 mechanism rotations
    public static final double POSITION_HIGH = 10.0;  // 10 mechanism rotations
  }
  
  public static class VisionConstants {
    // PhotonVision camera name - MUST match your camera name in PhotonVision
    public static final String CAMERA_NAME = "Test";  // Matches the camera found on NetworkTables
    
    // AprilTag IDs and corresponding positions
    public static final int TARGET_APRILTAG_ID = 15;  // The AprilTag ID to detect
    public static final double APRILTAG_TARGET_POSITION = 15.0;  // Position to go to when tag 15 is seen
  }
  
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
}
