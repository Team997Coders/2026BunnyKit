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
  private double fieldWidth = Units.inchesToMeters(315);
  private double fieldLength = Units.inchesToMeters(651);
  private List<Pose2d> centerPoses = new ArrayList<>();

  public GoToGrid(List<AprilTag> tags) {
    for (int i = 0; i < 8; i++) {
      Pose2d pos = tags.get(i).pose.toPose2d();
      Pose2d centerPose =
          // new Pose2d(
          //    pos.getX() + fieldLength / 2.0, pos.getY() + fieldWidth / 2.0, pos.getRotation());
          pos.plus(new Transform2d(-Units.inchesToMeters(15) / 2.0, 0.0, new Rotation2d()));
      // fieldLength / 2.0, fieldWidth / 2.0, new Rotation2d())); // center of cone
      targetPositions.add(centerPose);
      for (int r = 0; r < 4; r++) {
        targetPositions.add(
            centerPose.plus(
                new Transform2d(
                    Math.cos(r / 2.0 * Math.PI) * Units.inchesToMeters(15 / 2.0 + 16),
                    Math.sin(r / 2.0 * Math.PI) * Units.inchesToMeters(15 / 2.0 + 16),
                    new Rotation2d(r / 2.0 * Math.PI))));
      }
      m_field.getObject("Waypoints").setPoses(targetPositions);
      SmartDashboard.putData(m_field);
    }
  }

  public Pose2d getClosestPose(Pose2d startPose) {
    return startPose.nearest(targetPositions);
  }
}
