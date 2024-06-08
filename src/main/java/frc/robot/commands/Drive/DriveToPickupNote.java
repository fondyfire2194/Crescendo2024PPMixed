// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LLPipelines.pipelines;

public class DriveToPickupNote extends Command {
  /** Creates a new AlignToTagSetShootSpeed. */

  private final SwerveSubsystem m_swerve;
  private final TransferSubsystem m_transfer;
  private final IntakeSubsystem m_intake;
  private final String m_camname;
  private final boolean m_moveDirectionX;
  private final double m_maxDistance;
  double angleError = 0;
  private Timer elapsedTime = new Timer();
  double startPosition;
  private double distError;

  public DriveToPickupNote(
      SwerveSubsystem swerve,
      TransferSubsystem transfer,
      IntakeSubsystem intake,
      String camname,
      boolean moveDirectionX,
      double maxdistance) {
    m_swerve = swerve;
    m_transfer = transfer;
    m_intake = intake;
    m_camname = camname;
    m_moveDirectionX = moveDirectionX;
    m_maxDistance = maxdistance;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname, pipelines.NOTEDETECT1.ordinal());

    elapsedTime.reset();
    elapsedTime.start();

    SmartDashboard.putNumber("DtoPuN/swposnX", m_swerve.getX());
    SmartDashboard.putNumber("DtoPuN/swposnY", m_swerve.getY());

    if (m_moveDirectionX)
      startPosition = m_swerve.getX();
    else
      startPosition = m_swerve.getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotBase.isReal() && LimelightHelpers.getTV(m_camname)) {
      angleError = LimelightHelpers.getTX(m_camname);
      distError = LimelightHelpers.getTY(m_camname);
      SmartDashboard.putNumber("DtoPuN/AngErr", angleError);
    } else
      angleError = 0;

    double rotationVal = m_swerve.m_alignNotePID.calculate(angleError, 0);

    SmartDashboard.putNumber("DtoPuN/DistErr", distError);

    m_swerve.drive(
        -SwerveConstants.notePickupSpeed,
        0,
        rotationVal,
        false,
        true,
        false);

    if (AllianceUtil.isRedAlliance())
      m_swerve.remainingdistance = m_swerve.getX() - FieldConstants.FIELD_LENGTH / 2;
    else
      m_swerve.remainingdistance = FieldConstants.FIELD_LENGTH / 2 - m_swerve.getX();

    SmartDashboard.putNumber("DtoPuN/RmngDist", distError);

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
    return m_transfer.noteAtIntake() ||
        m_swerve.remainingdistance < -m_maxDistance;
  }
}
