// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert frontDisconnectedAlert;
  private final Alert backDisconnectedAlert;

  public Elevator(ElevatorIO io) {
    this.io = io;
    frontDisconnectedAlert = new Alert("Disconnected front motor on elevator.", AlertType.kError);
    backDisconnectedAlert = new Alert("Disconnected back motor on elevator.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update alerts
    frontDisconnectedAlert.set(!inputs.frontMotorConnected);
    backDisconnectedAlert.set(!inputs.backMotorConnected);
  }

  /**
   * Sets the target height and angle of the elevator
   *
   * @param height in meters, 0 is bottom
   * @param angle in radians, 0 is straight up, clockwise is positive
   */
  public void setPosition(double height, double angle) {
    io.setElevatorHeightMeters(height);
    io.setArmAngleRad(angle);
  }

  /**
   * Sets the target height and angle of the elevator
   *
   * @param height in meters, 0 is bottom
   * @param angle in radians, 0 is straight up, clockwise is positive
   */
  public void setPosition(double height, Rotation2d angle) {
    setPosition(height, angle.getRadians());
  }

  public Rotation2d getArmAngle() {
    return new Rotation2d(inputs.currentArmAngleRadians);
  }

  /** Returns the current height of the elevator in meters. */
  public double getHeight() {
    return inputs.currentHeightMeters;
  }
}
