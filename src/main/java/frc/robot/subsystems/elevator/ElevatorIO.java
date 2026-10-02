package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean frontMotorConnected = false;
    public boolean backMotorConnected = false;

    public double frontMotorPositionRad = 0.0;
    public double frontMotorVelocityRadPerSec = 0.0;
    public double frontMotorAppliedVolts = 0.0;
    public double frontMotorCurrentAmps = 0.0;

    public double backMotorPositionRad = 0.0;
    public double backMotorVelocityRadPerSec = 0.0;
    public double backMotorAppliedVolts = 0.0;
    public double backMotorCurrentAmps = 0.0;

    public double targetHeightTicks = 0.0;
    public double targetHeightMeters = 0.0;
    public double currentHeightMeters = 0.0;
    public double currentHeightTicks = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double elevatorFFvolts = 0.0;
    public double targetArmAngleRadians = 0.0;
    public double targetArmAngleTicks = 0.0;
    public double currentArmAngleRadians = 0.0;
    public double currentArmAngleTicks = 0.0;

    public double[] elevatorTimestamps = new double[] {};
    public double[] frontPositionsRad = new double[] {};
    public double[] backPositionsRad = new double[] {};
  }

  /** 
   * Update inputs (called every loop)
   * 
   * @param inputs
   */ 
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the target height of the elevator in meters, with 0 being bottomed out
   * @param height
   */
  public default void setElevatorHeightMeters(double height) {}

  /**
  * Sets the target height of the elevator in encoder ticks
  * You should probably set its position in meters and let 
  * the internal maths handle the encoder ticks
  * @param ticks
  */
  public default void setElevatorHeightEncoderTicks(double ticks) {}

  /**
   * Sets the target angle of the elevator arm in radians, 
   * with 0 going straight up and positive is clockwise
   * @param radians
   */
  public default void setArmAngleRad(double radians) {}
  
  /**
  * Sets the target angle of the elevator arm in encoder ticks
  * You should use setArmAngleRad and let 
  * the internal maths handle the encoder ticks
  * @param ticks
  */
  public default void setArmAngleEncoderTicks(double ticks) {}

  /**
   * @return height of elevator in meters, with 0 being bottomed out
   */
  public default double getHeightMeters() {
    return 0.0;
  }

  /** 
   * @return height of the elevator in encoder ticks, 
   * You should use the getHeightMeters and let
   * the internal maths handle the encoder ticks
   */ 
  public default double getHeightTicks() {
    return 0.0;
  }

  /**
   * @return angle of the elevator arm in radians, 
   * with 0 being straight up and positive is clockwise
   */
  public default double getArmAngleRadians() {
    return 0.0;
  }

  /** 
   * @return angle of the elevator arm in encoder ticks, 
   * You should use the getArmAngleRadians and let
   * the internal maths handle the encoder ticks
   */ 
  public default double getArmAngleTicks() {
    return 0.0;
  }
  
}
