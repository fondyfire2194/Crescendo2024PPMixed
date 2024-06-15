// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.LimelightHelpers;
import frc.robot.utils.LLPipelines.pipelines;
import monologue.Annotations.Log;
import monologue.Logged;

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
      LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname, pipelines.ND0CROP1.ordinal());

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

 
  public void setCamToRobotOffset(CameraConstants.CameraValues cam) {
    LimelightHelpers.setCameraPose_RobotSpace(cam.camname, cam.forward, cam.side, cam.up, cam.roll, cam.pitch, cam.yaw);
  }


  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

}