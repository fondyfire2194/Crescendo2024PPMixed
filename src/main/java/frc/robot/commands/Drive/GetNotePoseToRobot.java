// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LLPipelines.pipelines;

public class GetNotePoseToRobot extends Command {

  private final SwerveSubsystem m_swerve;

  private final LimelightVision m_llv;

  private final boolean m_noend;
  double angleError = 0;
  private Timer elapsedTime = new Timer();
  public int movingAverageNumTaps = 20;;
  public LinearFilter distanceFilter;
  double startPosition;

  private boolean onepass;

  private double rawDistanceToNote;

  private double distanceToNote;

  public GetNotePoseToRobot(
      SwerveSubsystem swerve,
      LimelightVision llv,
      boolean noend) {
    m_swerve = swerve;
    m_llv = llv;
    m_noend = noend;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    distanceFilter = LinearFilter.movingAverage(movingAverageNumTaps);

    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname, pipelines.NOTEDET1.ordinal());

    elapsedTime.reset();
    elapsedTime.start();
    onepass = false;
    m_swerve.notePoseCalculated = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotBase.isReal() && LimelightHelpers.getTV(m_llv.rname)) {

      double boundingHorizontalPixels = m_llv.getBoundingHorizontalPixels();// get width of note seen
      // estimated distance in meters from camera geometry
       rawDistanceToNote = m_llv.distanceFromCameraPercentage(boundingHorizontalPixels);
      // average distance reading over 20 reads
      distanceToNote = distanceFilter.calculate(rawDistanceToNote);

      Transform2d noteToCamera = m_llv.getNoteTransform2dFromCamera(distanceToNote);

      m_swerve.targetPose = m_swerve.getPose().transformBy(noteToCamera);

      m_swerve.setPathfindPose(m_swerve.targetPose);

    } else {

      m_swerve.setPathfindPose(m_swerve.targetPose);
      onepass = true;
      m_swerve.notePoseCalculated = true;
    }

    SmartDashboard.putNumber("NoteDist", distanceToNote);
    SmartDashboard.putNumber("NoteDistRaw", rawDistanceToNote);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotBase.isSimulation() && onepass || !m_noend && elapsedTime.hasElapsed(3);
  }
}
