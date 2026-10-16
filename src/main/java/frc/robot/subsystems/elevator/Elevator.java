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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final LoggedMechanism2d mechCanvas = new LoggedMechanism2d(2.0, 2.0);
  private final LoggedMechanismRoot2d mechRoot;
  private final LoggedMechanismLigament2d armLigament;
  private final LoggedMechanismLigament2d simElevator;

  private final Alert frontDisconnectedAlert;
  private final Alert backDisconnectedAlert;

  public Elevator(ElevatorIO io) {
    this.io = io;

    mechRoot = mechCanvas.getRoot("elevatorBase", 1.0, 0.5);
    simElevator =
        mechRoot.append(
            new LoggedMechanismLigament2d(
                "simElevator", 0.7, 0.0, 7, new Color8Bit(Color.kAliceBlue)));
    armLigament =
        simElevator.append(
            new LoggedMechanismLigament2d(
                "ArmSegment", 0.4, 0.0, 6, new Color8Bit(Color.kCrimson)));
    simElevator.setAngle(90);

    frontDisconnectedAlert = new Alert("Disconnected front motor on elevator.", AlertType.kError);
    backDisconnectedAlert = new Alert("Disconnected back motor on elevator.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    armLigament.setAngle(Math.toDegrees(inputs.currentArmAngleRadians));
    simElevator.setLength(inputs.currentHeightMeters + 1);

    // 5. Explicitly update the data stream to AdvantageKit every loop cycle
    Logger.recordOutput("Mechanisms/Arm2D", mechCanvas);

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
