// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.GeometryUtil;
import frc.robot.utils.LLPipelines.pipelines;

public class DriveToPickupNote extends Command {
  /** Creates a new AlignToTagSetShootSpeed. */

  private final SwerveSubsystem m_swerve;
  private final TransferSubsystem m_transfer;
  private final IntakeSubsystem m_intake;
  private final String m_camname;
  private final LimelightVision m_llv;
  private final int m_noteNumber;
  double angleError = 0;
  private Timer elapsedTime = new Timer();
  double endPosition;
  private double xerror;

  private Pose2d activeNotePose = new Pose2d();
  private double maxYDistance = Units.inchesToMeters(60);
  private boolean redAlliance;

  public DriveToPickupNote(
      SwerveSubsystem swerve,
      TransferSubsystem transfer,
      IntakeSubsystem intake,
      String camname,
      LimelightVision llv,
      int noteNumber) {
    m_swerve = swerve;
    m_transfer = transfer;
    m_intake = intake;
    m_camname = camname;
    m_llv = llv;
    m_noteNumber = noteNumber;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname, pipelines.NOTE_DETECT8.ordinal());

    elapsedTime.reset();
    elapsedTime.start();

    activeNotePose = AllianceUtil.flipFieldAngle(Constants.getActiveNotePickup(m_noteNumber));

    SmartDashboard.putString("NotePose", activeNotePose.toString());

    double xStartPosition = m_swerve.getX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotBase.isReal() && LimelightHelpers.getTV(m_camname)) {

      angleError = LimelightHelpers.getTX(m_camname);

      SmartDashboard.putNumber("NOTEAngErr", angleError);

    } else

      angleError = 0;

    double rotationVal = m_swerve.m_alignNotePID.calculate(angleError, 0);

    if (redAlliance)
      xerror = m_swerve.getX() - activeNotePose.getX();
    else
      xerror = activeNotePose.getX() - m_swerve.getX();

    SmartDashboard.putNumber("TAXerr", xerror);

    /*
     * Drive
     * assumes robot is pointing in y plane to pickup a center note;
     * could be a higher or lower y value note
     */

    m_swerve.drive(
        -SwerveConstants.notePickupSpeed * xerror * 3, // Constants.SwerveConstants.kmaxSpeed *3,
        0,
        rotationVal,
        false,
        true,
        false);
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
    return Math.abs(FieldConstants.FIELD_LENGTH / 2 - m_swerve.getX()) < .2 && Math.abs(xerror) < .1
        || RobotBase.isReal() && m_transfer.noteAtIntake();
  }
}
