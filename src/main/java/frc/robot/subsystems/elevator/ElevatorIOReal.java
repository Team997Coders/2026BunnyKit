package frc.robot.subsystems.elevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ElevatorIOReal implements ElevatorIO {
    public SparkMax motor1;
    public SparkMax motor2; 
    public Canandmag motor1encoder;
    public Canandmag motor2encoder;
    public Canandmag armencoder;


     private PIDController frontController =
      new PIDController(elevatorSimP, elevatorSimI, elevatorSimD);
  private PIDController backController =
      new PIDController(elevatorSimP, elevatorSimI, elevatorSimD);
  private double elevatorFFvolts =
      0.0; // Assuming FF volts is going to be the bare minimum to keep the elevator from dropping
  private double frontAppliedVolts = 0.0;
  private double backAppliedVolts = 0.0;

  private double targetHeightTicks = 0.0;
  private double targetAngleTicks = 0.0;
    public ElevatorIOReal() {
        motor1 = new SparkMax(ElevatorConstants.motor1ID, MotorType.kBrushless);
        motor2 = new SparkMax(ElevatorConstants.motor2ID, MotorType.kBrushless);
        motor1encoder = new Canandmag(endcoder1ID);
        motor2encoder = new Canandmag(encoder2ID);
        armencoder = new Canandmag(armcoderID);


    }

 @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    frontController.setSetpoint(targetAngleTicks + targetHeightTicks);
    backController.setSetpoint(targetAngleTicks - targetHeightTicks);

    frontAppliedVolts =
        elevatorFFvolts
            + frontController.calculate(motor1encoder.getAbsPosition() * ticksPerRadianMotor);
    backAppliedVolts =
        -elevatorFFvolts
            + backController.calculate(motor2encoder.getAbsPosition() * ticksPerRadianMotor);


    motor1.setVoltage(MathUtil.clamp(frontAppliedVolts, -12.0, 12.0));
    motor2.setVoltage(MathUtil.clamp(backAppliedVolts, -12.0, 12.0));


    inputs.frontMotorAppliedVolts = frontAppliedVolts;
    inputs.backMotorAppliedVolts = backAppliedVolts;
    inputs.frontMotorConnected = true;
    inputs.backMotorConnected = true;
    inputs.frontMotorCurrentAmps = motor1.getOutputCurrent();
    inputs.backMotorCurrentAmps = motor2.getOutputCurrent();
    inputs.frontMotorPositionRad = motor1encoder.getAbsPosition();
    inputs.backMotorPositionRad = motor2encoder.getAbsPosition();
    inputs.frontMotorVelocityRadPerSec = motor1encoder.getVelocity();
    inputs.backMotorVelocityRadPerSec = motor2encoder.getVelocity();

    inputs.targetHeightMeters = targetHeightTicks / ticksPerMeter;
    inputs.targetHeightTicks = targetHeightTicks;
    inputs.currentHeightMeters = getHeightMeters();
    inputs.currentHeightTicks = getHeightTicks();
    inputs.targetArmAngleRadians = targetAngleTicks / ticksPerRadianArm;
    inputs.targetArmAngleTicks = targetAngleTicks;
    inputs.currentArmAngleRadians = getArmAngleRadians();
    inputs.currentArmAngleTicks = getArmAngleTicks();

    inputs.elevatorTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.frontPositionsRad = new double[] {inputs.frontMotorPositionRad};
    inputs.backPositionsRad = new double[] {inputs.backMotorPositionRad};
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

  
  public double getHeightMeters() {
    return getHeightTicks() / ticksPerMeter;
  }

  public double getHeightTicks() {
    return (motor1encoder.getAbsPosition() - motor2encoder.getAbsPosition())
        * ticksPerRadianMotor;
  }

  public double getArmAngleRadians() {
    return getArmAngleTicks() / ticksPerRadianArm;
  }

  public double getArmAngleTicks() {
    return (motor1encoder.getAbsPosition() + motor2encoder.getAbsPosition())
        * ticksPerRadianMotor;
  }
  

}