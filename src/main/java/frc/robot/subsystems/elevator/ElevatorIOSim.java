package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorIOSim implements ElevatorIO {
  private final DCMotorSim frontSim;
  private final DCMotorSim backSim;

  public PIDController frontController =
      new PIDController(elevatorSimP, elevatorSimI, elevatorSimD);
  public PIDController backController = new PIDController(elevatorSimP, elevatorSimI, elevatorSimD);

  private double targetHeightTicks = 0.0;
  private double targetAngleTicks = 0.0;

  private final LoggedMechanism2d mechCanvas = new LoggedMechanism2d(2.0, 2.0);
  private final LoggedMechanismRoot2d mechRoot;
  private final LoggedMechanismLigament2d armLigament;
  private final LoggedMechanismLigament2d simElevator;

  public ElevatorIOSim() {
    frontSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(frontGearbox, 0.00036, frontMotorReduction),
            frontGearbox);
    backSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(backGearbox, 0.00036, backMotorReduction),
            backGearbox);

    mechRoot = mechCanvas.getRoot("elevatorBase", 1.0, 0.5);
    simElevator =
        mechRoot.append(
            new LoggedMechanismLigament2d(
                "simElevator", 0.0, 0.0, 7, new Color8Bit(Color.kAliceBlue)));
    armLigament =
        simElevator.append(
            new LoggedMechanismLigament2d(
                "ArmSegment", 0.7, 0.0, 6, new Color8Bit(Color.kCrimson)));
    simElevator.setAngle(90);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    frontController.setSetpoint(inputs.targetArmAngleTicks + inputs.targetHeightTicks);
    backController.setSetpoint(inputs.targetArmAngleTicks - inputs.targetHeightTicks);

    inputs.frontMotorAppliedVolts =
        inputs.elevatorFFvolts
            + frontController.calculate(frontSim.getAngularPositionRad() * ticksPerRadianMotor);
    inputs.backMotorAppliedVolts =
        -inputs.elevatorFFvolts
            + backController.calculate(backSim.getAngularPositionRad() * ticksPerRadianMotor);

    // Update simulation state
    frontSim.setInputVoltage(MathUtil.clamp(inputs.frontMotorAppliedVolts, -12.0, 12.0));
    backSim.setInputVoltage(MathUtil.clamp(inputs.backMotorAppliedVolts, -12.0, 12.0));
    frontSim.update(0.02);
    backSim.update(0.02);

    inputs.frontMotorConnected = true;
    inputs.backMotorConnected = true;
    inputs.frontMotorCurrentAmps = frontSim.getCurrentDrawAmps();
    inputs.backMotorCurrentAmps = backSim.getCurrentDrawAmps();
    inputs.frontMotorPositionRad = frontSim.getAngularPositionRad();
    inputs.backMotorPositionRad = backSim.getAngularPositionRad();
    inputs.frontMotorVelocityRadPerSec = frontSim.getAngularVelocityRadPerSec();
    inputs.backMotorVelocityRadPerSec = backSim.getAngularVelocityRadPerSec();

    inputs.targetHeightMeters = targetHeightTicks / ticksPerMeter;
    inputs.targetHeightTicks = targetHeightTicks;
    inputs.currentHeightMeters = getHeightMeters();
    inputs.currentHeightTicks = getHeightTicks();
    inputs.targetArmAngleRadians = targetAngleTicks / ticksPerRadianArm;
    inputs.targetArmAngleTicks = targetAngleTicks;
    inputs.currentArmAngleRadians = getArmAngleRadians();
    inputs.currentArmAngleTicks = getArmAngleTicks();

    // inputs.elevatorTimestamps = new double[] {Timer.getFPGATimestamp()};
    // inputs.frontPositionsRad = new double[] {inputs.frontMotorPositionRad};
    // inputs.backPositionsRad = new double[] {inputs.backMotorPositionRad};

    armLigament.setAngle(Math.toDegrees(inputs.currentArmAngleRadians));
    simElevator.setLength(inputs.currentHeightMeters + 0.2);

    Logger.recordOutput("Mechanisms/Arm2D", mechCanvas);
  }

  public void setElevatorHeightMeters(double height) {
    setElevatorHeightEncoderTicks(height * ticksPerMeter);
  }

  public void setElevatorHeightEncoderTicks(double ticks) {
    targetHeightTicks = ticks;
  }

  public void setArmAngleRad(double radians) {
    setArmAngleEncoderTicks(radians * ticksPerRadianArm);
  }

  public void setArmAngleEncoderTicks(double ticks) {
    targetAngleTicks = ticks;
  }

  public void setElevatorState(ElevatorState state) {
    setElevatorHeightMeters(state.getHeight());
    setArmAngleRad(state.getAngle());
  }

  public double getHeightMeters() {
    return getHeightTicks() / ticksPerMeter;
  }

  public double getHeightTicks() {
    return (frontSim.getAngularPositionRad() - backSim.getAngularPositionRad())
        * ticksPerRadianMotor;
  }

  public double getArmAngleRadians() {
    return getArmAngleTicks() / ticksPerRadianArm;
  }

  public double getArmAngleTicks() {
    return (frontSim.getAngularPositionRad() + backSim.getAngularPositionRad())
        * ticksPerRadianMotor;
  }
}
