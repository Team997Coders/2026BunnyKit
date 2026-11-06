package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class goTo extends Command {
  private Drive drive;
  private Pose2d goalPos;
  private Pose2d startPos;
  private ChassisSpeeds goalSpeeds;
  private ChassisSpeeds startingSpeeds;
  private Timer timer;
  private double T;
  private TrapezoidProfile.Constraints constraints =
      new Constraints(Math.PI, Math.pow(Math.PI, 2.0));
  private ProfiledPIDController rController = new ProfiledPIDController(4, 0, 0, constraints);

  public goTo(Drive drive, Pose2d goalPos) {
    this.drive = drive;
    this.goalPos = goalPos;
    this.goalSpeeds = new ChassisSpeeds();
  }

  public goTo(Drive drive, Pose2d goalPos, ChassisSpeeds goalSpeeds) {
    this.drive = drive;
    this.goalPos = goalPos;
    this.goalSpeeds = goalSpeeds;
  }

  @Override
  public void initialize() {
    this.startingSpeeds = drive.getChassisSpeeds();
    this.startPos = drive.getPose();
    rController.reset(this.startPos.getRotation().getRadians());
    rController.setGoal(this.goalPos.getRotation().getRadians());
    timer = new Timer();
    timer.start();
    T = 2;
  }

  @Override
  public void execute() {
    double t = timer.get();

    double xf = this.goalPos.getX();
    double vxf = this.goalSpeeds.vxMetersPerSecond;
    double x0 = this.startPos.getX();
    double ax1 = this.startingSpeeds.vxMetersPerSecond;
    double ax2 = 0;
    double ax3 = (20 * (xf - x0) - (8 * vxf + 12 * ax1) * T) / (2 * Math.pow(T, 3.0));
    double ax4 = (-30 * (xf - x0) + (14 * vxf + 16 * ax1) * T) / (2 * Math.pow(T, 4.0));
    double ax5 = (12 * (xf - x0) - (6 * vxf + 6 * ax1) * T) / (2 * Math.pow(T, 5.0));
    double vx =
        ax1
            + 2 * ax2 * t
            + 3 * ax3 * Math.pow(t, 2.0)
            + 4 * ax4 * Math.pow(t, 3.0)
            + 5 * ax5 * Math.pow(t, 4.0);

    double yf = this.goalPos.getY();
    double vyf = this.goalSpeeds.vyMetersPerSecond;
    double y0 = this.startPos.getY();
    double ay1 = this.startingSpeeds.vyMetersPerSecond;
    double ay2 = 0;
    double ay3 = (20 * (yf - y0) - (8 * vyf + 12 * ay1) * T) / (2 * Math.pow(T, 3.0));
    double ay4 = (-30 * (yf - y0) + (14 * vyf + 16 * ay1) * T) / (2 * Math.pow(T, 4.0));
    double ay5 = (12 * (yf - y0) - (6 * vyf + 6 * ay1) * T) / (2 * Math.pow(T, 5.0));
    double vy =
        ay1
            + 2 * ay2 * t
            + 3 * ay3 * Math.pow(t, 2.0)
            + 4 * ay4 * Math.pow(t, 3.0)
            + 5 * ay5 * Math.pow(t, 4.0);

    double r = drive.getPose().getRotation().getRadians();
    double vr = rController.calculate(r);

    SmartDashboard.putNumber("vx", vx);
    SmartDashboard.putNumber("vy", vy);
    SmartDashboard.putNumber("vr", vr);

    double fx = vx * Math.cos(r) - vy * Math.sin(r);
    double fy = vy * Math.cos(r) + vx * Math.sin(r);

    SmartDashboard.putNumber("fx", fx);
    SmartDashboard.putNumber("fy", fy);
    ChassisSpeeds vChassisSpeeds = new ChassisSpeeds(fx, fy, vr);
    drive.runVelocity(vChassisSpeeds);
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= T;
  }
}
