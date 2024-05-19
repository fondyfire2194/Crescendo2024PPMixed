package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;


import java.util.Optional;

public class AllianceUtil {
  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Red;
    } else {
      return false;
    }
  }

  public static Rotation2d getZeroRotation() {
    if (isRedAlliance()) {
      return Rotation2d.fromDegrees(180.0);
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }

  public static Pose2d flipFieldAngle(Pose2d pose) {
    Translation2d t2d = new Translation2d();
    if (isRedAlliance()) {
      t2d = pose.getTranslation();
      double rads = pose.getRotation().getRadians();
      return new Pose2d(t2d, new Rotation2d(rads + Math.PI));
    } else
      return pose;
  }

  public static Pose2d getSpeakerPose() {
    return isRedAlliance() ? FieldConstants.speakerRedAlliance : FieldConstants.speakerBlueAlliance;
  }

  public static Pose2d getLobPose() {
    return isRedAlliance() ? FieldConstants.lobRedAlliance : FieldConstants.lobBlueAlliance;
  }

  public static Pose2d getStagePose() {
    return isRedAlliance() ? FieldConstants.stageRedAlliance : FieldConstants.stageBlueAlliance;
  }
}
