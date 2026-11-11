package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorConstants {
  public static final double elevatorSimP = 0.5;
  public static final double elevatorSimI = 0.0;
  public static final double elevatorSimD = 0.0;
  public static final double elevatorRealP = 0.5;
  public static final double elevatorRealI = 0.0;
  public static final double elevatorRealD = 0.0;

  public static final DCMotor frontGearbox = DCMotor.getNEO(1);
  public static final DCMotor backGearbox = DCMotor.getNEO(1);

  public static final double frontMotorReduction = 6.75;
  public static final double backMotorReduction = 6.75;

  // Making up numbers :)
  public static final double maxExtensionMeters = 0.7;
  public static final double maxExtensionEncoderTicks = 2234;
  public static final double ticksPerMeter = maxExtensionEncoderTicks / maxExtensionMeters;

  public static final double fullRotationTicks = 1302;
  public static final double ticksPerRadianArm = fullRotationTicks / Math.PI / 2;

  public static final double ticksPerRotation = 42; // real number
  public static final double ticksPerRadianMotor = ticksPerRotation / Math.PI / 2;

  public static final double pickUpStandingHeight = 0;
  public static final double toppledAngle = Math.PI / 2 + 0.6;
  public static final double pickUpToppledHeight = 0.05;
  public static final double scoreLowHeight = 0.3;
  public static final double scoreHighHeight = 0.45;

  public static int motor1ID = 0; // TODO: get real can id's
  public static int motor2ID = 0;
  public static int endcoder1ID = 0;
  public static int encoder2ID = 0;
  public static int armcoderID = 0;
}
