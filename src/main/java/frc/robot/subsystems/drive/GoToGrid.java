package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

public class GoToGrid {
  private List<Pose2d> targetPositions = new ArrayList<>();
  private final Field2d m_field = new Field2d();

  public GoToGrid(List<AprilTag> tags) {
    for (AprilTag tag : tags) {

    Pose2d pos = tag.pose.toPose2d();
      Pose2d centerPose =
          pos.plus(
              new Transform2d(
                  -Math.cos(pos.getRotation().getRadians()) * Units.inchesToMeters(15),
                  -Math.sin(pos.getRotation().getRadians()) * Units.inchesToMeters(15),
                  new Rotation2d())); // center of cone
      for (int r = 0; r < 2 * Math.PI; r += Math.PI / 2.0) {
        targetPositions.add(
            centerPose.plus(
                new Transform2d(
                    Math.cos(r) * Units.inchesToMeters(15 + 14),
                    Math.sin(r) * Units.inchesToMeters(15 + 14),
                    new Rotation2d())));
      }
      m_field.getObject("Waypoints").setPoses(targetPositions);
      SmartDashboard.putData(m_field);
    }
  }

  public Pose2d getClosestPose(Pose2d startPose) {
    return startPose.nearest(targetPositions);
  }
}
