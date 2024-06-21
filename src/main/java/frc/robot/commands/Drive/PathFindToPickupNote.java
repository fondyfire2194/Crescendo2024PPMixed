// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.LLPipelines.pipelines;

public class PathFindToPickupNote extends Command {
  /** Creates a new AlignToTagSetShootSpeed. */

  private final SwerveSubsystem m_swerve;
  private final TransferSubsystem m_transfer;
  private final IntakeSubsystem m_intake;
  private final LimelightVision m_llv;
  private final String m_camname;
  double angleError = 0;
  private Timer elapsedTime = new Timer();
  public int movingAverageNumTaps = 20;;
  public LinearFilter distanceFilter;
  double startPosition;
  private int m_centerNoteNumber;

  Pose2d targetNotePose = new Pose2d();
  private boolean onepass;

  public PathFindToPickupNote(
      SwerveSubsystem swerve,
      TransferSubsystem transfer,
      IntakeSubsystem intake,
      LimelightVision llv,
      int centerNoteNumber,
      String camname) {
    m_swerve = swerve;
    m_transfer = transfer;
    m_intake = intake;
    m_llv = llv;
    m_centerNoteNumber = centerNoteNumber;
    m_camname = camname;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    distanceFilter = LinearFilter.movingAverage(movingAverageNumTaps);

    targetNotePose = FieldConstants.centerNotes[m_centerNoteNumber];
    m_swerve.targetPose = targetNotePose;

    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname, pipelines.NOTEDET1.ordinal());

    elapsedTime.reset();
    elapsedTime.start();
    onepass = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotBase.isReal() && LimelightHelpers.getTV(m_camname)) {

      double boundingHorizontalPixels = m_llv.getBoundingHorizontalPixels();// get width of note seen
      // estimated distance in meters from camera geometry
      double rawDistanceToNote = m_llv.distanceFromCameraPercentage(boundingHorizontalPixels);
      // average distance reading over 20 reads
      double distanceToNote = distanceFilter.calculate(rawDistanceToNote);
      Transform2d noteToCamera = m_llv.getNoteTransform2dFromCamera(distanceToNote);

      m_swerve.targetPose = m_swerve.getPose().transformBy(noteToCamera);

      m_swerve.setPathfindPose(m_swerve.targetPose);

    } else {

      m_swerve.setPathfindPose(m_swerve.targetPose);
      onepass = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(0, 0, 0, false, true, false);
    m_transfer.stopMotor();
    m_intake.stopMotor();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotBase.isSimulation() && onepass || elapsedTime.hasElapsed(1);
  }
}
