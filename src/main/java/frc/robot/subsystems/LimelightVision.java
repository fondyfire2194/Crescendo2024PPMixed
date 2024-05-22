// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.ArrayList;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.LimelightHelpers;
import frc.robot.utils.LLPipelines;
import monologue.Logged;
import monologue.Annotations.Log;

public class LimelightVision extends SubsystemBase implements Logged {
  /** Creates a new LimelightVision. */

  private double llHeartbeatfl;
  private double llHeartbeatLastfl;
  private int samplesfl;
  @Log.NT(key = "flexists")
  public boolean limelightExistsfl;

  private double llHeartbeatfr;
  private double llHeartbeatLastfr;
  private int samplesfr;
  @Log.NT(key = "frexists")
  public boolean limelightExistsfr;

  private double llHeartbeatr;
  private double llHeartbeatLastr;
  private int samplesr;
  @Log.NT(key = "rexists")
  public boolean limelightExistsr;
  private int loopctr;

  public String flname = CameraConstants.frontLeftCamera.camname;
  public String frname = CameraConstants.frontRightCamera.camname;
  public String rname = CameraConstants.rearCamera.camname;

  Optional<Pose3d> temp;

  public LimelightVision() {

    if (CameraConstants.rearCamera.isUsed)
      setRearNoteDetectorPipeline();

    if (CameraConstants.frontLeftCamera.isUsed)
      setCamToRobotOffset(CameraConstants.frontLeftCamera);

    if (CameraConstants.frontRightCamera.isUsed)
      setCamToRobotOffset(CameraConstants.frontRightCamera);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (loopctr > 2)
      loopctr = 0;

    if (RobotBase.isReal()) {
      if (CameraConstants.frontLeftCamera.isUsed && loopctr == 0) {
        llHeartbeatfl = LimelightHelpers.getLimelightNTDouble(CameraConstants.frontLeftCamera.camname, "hb");
        if (llHeartbeatfl == llHeartbeatLastfl) {
          samplesfl += 1;
        } else {
          samplesfl = 0;
          llHeartbeatLastfl = llHeartbeatfl;
          limelightExistsfl = true;
        }
        if (samplesfl > 5)
          limelightExistsfl = false;

        CameraConstants.frontLeftCamera.isActive = limelightExistsfl;
      }
      if (CameraConstants.frontRightCamera.isUsed && loopctr == 1) {
        llHeartbeatfr = LimelightHelpers.getLimelightNTDouble(CameraConstants.frontRightCamera.camname, "hb");
        if (llHeartbeatfr == llHeartbeatLastfr) {
          samplesfr += 1;
        } else {
          samplesfr = 0;
          llHeartbeatLastfr = llHeartbeatfr;
          limelightExistsfr = true;
        }
        if (samplesfr > 5)
          limelightExistsfr = false;

        CameraConstants.frontRightCamera.isActive = limelightExistsfr;
      }

      if (CameraConstants.rearCamera.isUsed && loopctr == 2) {
        llHeartbeatr = LimelightHelpers.getLimelightNTDouble(CameraConstants.rearCamera.camname, "hb");
        if (llHeartbeatr == llHeartbeatLastr) {
          samplesr += 1;
        } else {
          samplesr = 0;
          llHeartbeatLastr = llHeartbeatr;
          limelightExistsr = true;
        }
        if (samplesr > 5)
          limelightExistsr = false;

        CameraConstants.rearCamera.isActive = limelightExistsr;
      }
    }

    loopctr++;

    SmartDashboard.putBoolean("LL//FrontLeftCamOk", limelightExistsfl);
    SmartDashboard.putBoolean("LL//FrontRightCamOk", limelightExistsfr);
    SmartDashboard.putBoolean("LL//RearCamOk", limelightExistsr);

    boolean allcamsok = CameraConstants.frontLeftCamera.isUsed && limelightExistsfl
        && CameraConstants.frontRightCamera.isUsed && limelightExistsfr
        && CameraConstants.rearCamera.isUsed && limelightExistsr;
    SmartDashboard.putBoolean("LL//CamsOK", allcamsok);
  }

  public void setAprilTag_ALL_Pipeline() {
    LimelightHelpers.setPipelineIndex(CameraConstants.frontLeftCamera.camname,
        LLPipelines.pipelines.APRILTAGALL0.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.frontRightCamera.camname,
        LLPipelines.pipelines.APRILTAGALL0.ordinal());
  }

  public void setAprilTagStartPipeline() {
    LimelightHelpers.setPipelineIndex(CameraConstants.frontLeftCamera.camname,
        LLPipelines.pipelines.APRILTAGSTART1.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.frontRightCamera.camname,
        LLPipelines.pipelines.APRILTAGSTART1.ordinal());
  }

  public void setRearNoteDetectorPipeline() {
    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname,
        LLPipelines.pipelines.NOTE_DETECT8.ordinal());
  }

  public void setAlignSpeakerPipeline() {
    LimelightHelpers.setPipelineIndex(CameraConstants.frontLeftCamera.camname,
        LLPipelines.pipelines.APRILTAGALIGN5.ordinal());
  }

  public int getNumberTagsSeen(CameraConstants.CameraValues cam) {
    return (int) LimelightHelpers
        .getLatestResults(cam.camname).targetingResults.targets_Fiducials.length;
  }

  public String getTagsSeen(CameraConstants.CameraValues cam) {
    var temp = LimelightHelpers.getLatestResults(cam.camname).targetingResults;

    var temp1 = temp.targets_Fiducials;
    int l = temp1.length;
    if (l <= 0)
      return String.valueOf(0);
    if (l == 1)
      return String.valueOf((int) temp1[0].fiducialID);
    if (l >= 2)
      return String.valueOf((int) temp1[0].fiducialID) + " , " + String.valueOf((int) temp1[1].fiducialID);
    else
      return "Problem";
  }
  @Log.NT(key = "notesseenqty")
  public int getNumberNotesSeen() {
    return (int) LimelightHelpers
        .getLatestResults(CameraConstants.rearCamera.camname).targetingResults.targets_Detector.length;
  }

  @Log.NT(key = "noteseen")
  public boolean getNoteSeen() {
    return LimelightHelpers.getTV(CameraConstants.rearCamera.camname);
  }
  @Log.NT(key = "noteseenangle")
  public double getNoteSeenTx() {
    return LimelightHelpers.getTX(CameraConstants.rearCamera.camname);
  }

  public int getTagId(CameraConstants.CameraValues cam) {
    return (int) LimelightHelpers.getFiducialID(cam.camname);
  }

  public boolean getTV(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getTV(cam.camname);
  }

  public Pose3d getTagPose3d(CameraConstants.CameraValues cam) {
    int tagID = (int) LimelightHelpers.getFiducialID(cam.camname);
    Optional<Pose3d> aprilTagPose = Constants.AprilTagConstants.layout.getTagPose(tagID);
    if (aprilTagPose.isPresent())
      return aprilTagPose.get();
    else
      return new Pose3d();
  }

  public Pose3d getAnyTagPose3d(int tagID) {
    Optional<Pose3d> aprilTagPose = Constants.AprilTagConstants.layout.getTagPose(tagID);
    if (aprilTagPose.isPresent())
      return aprilTagPose.get();
    else
      return new Pose3d();
  }

  public double getDistanceFromTag(CameraConstants.CameraValues cam) {
    int tagID = (int) LimelightHelpers.getFiducialID(cam.camname);
    return getTranslationFromTag(cam, tagID).getNorm();
  }

  public double getDistanceFromSpeakerTag(CameraConstants.CameraValues cam) {
    int tagID = (int) LimelightHelpers.getFiducialID(cam.camname);
    return getTranslationFromTag(cam, tagID).getNorm();
  }

  public boolean hasTarget(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getTV(cam.camname);
  }

  public double[] getTargetPoseRobotSpaceAsDoubles(CameraConstants.CameraValues cam, int id) {
    double[] temp = new double[6];
    if (LimelightHelpers.getTV(cam.camname) && LimelightHelpers.getFiducialID(cam.camname) == id)
      return LimelightHelpers.getTargetPose_RobotSpace(cam.camname);
    else
      return temp;
  }

  public double[] getCameraPoseTargetSpaceasDoubles(CameraConstants.CameraValues cam, int id) {
    double[] temp = new double[6];
    if (LimelightHelpers.getTV(cam.camname) && LimelightHelpers.getFiducialID(cam.camname) == id)
      return LimelightHelpers.getCameraPose_TargetSpace(cam.camname);
    else
      return temp;
  }

  public double[] getTargetPoseCameraSpace(CameraConstants.CameraValues cam, int id) {
    double[] temp = new double[6];
    if (LimelightHelpers.getTV(cam.camname) && LimelightHelpers.getFiducialID(cam.camname) == id)
      return LimelightHelpers.getTargetPose_CameraSpace(cam.camname);
    else
      return temp;
  }

  public Pose3d getCameraPoseTargetSpace3d(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getCameraPose3d_RobotSpace(cam.camname);
  }

  public double[] getBotPoseWPI_BlueAsDoubles(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getBotPose_wpiBlue(cam.camname);
  }

  public double[] getBotPoseWPI_RedAsDoubles(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getBotPose_wpiRed(cam.camname);
  }

  public Pose3d getBotPose3d(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getBotPose3d(cam.camname);
  }

  public Pose3d getBotPose3dTargetSpace(CameraConstants.CameraValues cam) {
    return LimelightHelpers.getBotPose3d_TargetSpace(cam.camname);
  }

  public Pose3d getTagPose(CameraConstants.CameraValues cam) {
    int tagID = (int) LimelightHelpers.getFiducialID(cam.camname);
    // SmartDashboard.putNumber("SHID", tagID);
    Optional<Pose3d> aprilTagPose = Constants.AprilTagConstants.layout.getTagPose(tagID);
    if (aprilTagPose.isPresent())
      return aprilTagPose.get();
    else
      return new Pose3d();
  }

  public void setCamToRobotOffset(CameraConstants.CameraValues cam) {
    LimelightHelpers.setCameraPose_RobotSpace(cam.camname, cam.forward, cam.side, cam.up, cam.roll, cam.pitch, cam.yaw);
  }

  public Translation2d getTranslationFromTag(CameraConstants.CameraValues cam, int tagid) {

    Optional<Pose3d> aprilTagPose = Constants.AprilTagConstants.layout.getTagPose(tagid);

    if (aprilTagPose.isPresent()) {

      Pose2d tagPose2d = aprilTagPose.get().toPose2d();

      // // get botpose from limelight networktables

      Pose2d botPose2d = LimelightHelpers.getBotPose2d_wpiBlue(cam.camname);

      Translation2d botToTagTranslation2d = botPose2d.getTranslation().minus(tagPose2d.getTranslation());

      return botToTagTranslation2d;

    } else

      return new Translation2d();
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

  public Translation2d getAllianceSpeakerTranslation() {
    int tagid = 3;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() &&
        alliance.get() == DriverStation.Alliance.Blue)
      tagid = 7;
    return getAnyTagPose3d(tagid).toPose2d().getTranslation();

  }
}