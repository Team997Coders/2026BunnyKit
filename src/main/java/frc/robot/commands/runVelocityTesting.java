package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class runVelocityTesting extends Command {
  private Drive drive;

  public runVelocityTesting(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    ChassisSpeeds vChassisSpeeds = new ChassisSpeeds(1, 0, 0);
    drive.runVelocity(vChassisSpeeds);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
