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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LLPipelines.pipelines;

public class DriveToPickupNote extends Command {
  /** Creates a new AlignToTagSetShootSpeed. */
  private final SwerveSubsystem m_swerve;
  private final TransferSubsystem m_transfer;
  private final IntakeSubsystem m_intake;

  double angleError = 0;
  private Timer elapsedTime = new Timer();
  private boolean toofar;
  private double llyangle;
  private String m_camname = CameraConstants.rearCamera.camname;

  public DriveToPickupNote(
      SwerveSubsystem swerve,
      TransferSubsystem transfer,
      IntakeSubsystem intake) {

    m_swerve = swerve;
    m_transfer = transfer;
    m_intake = intake;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname, pipelines.NOTEDET1.ordinal());

    elapsedTime.reset();
    elapsedTime.start();

    SmartDashboard.putNumber("DtoPuN/swposnX", m_swerve.getX());
    SmartDashboard.putNumber("DtoPuN/swposnY", m_swerve.getY());
    int nn = m_swerve.targetNote;
    if (nn < 0)
      nn = 0;
    if (nn > 5)
      nn = 5;
    m_swerve.targetPose = FieldConstants.centerNotes[nn];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotBase.isReal() && LimelightHelpers.getTV(m_camname)) {
      angleError = LimelightHelpers.getTX(m_camname);
      llyangle = LimelightHelpers.getTY(m_camname);
      SmartDashboard.putNumber("DtoPuN/AngErr", angleError);
    } else
      angleError = 0;

    double rotationVal = m_swerve.m_alignNotePID.calculate(angleError, 0);

    SmartDashboard.putNumber("DtoPuN/LLYangle", llyangle);

    m_swerve.drive(
        -SwerveConstants.notePickupSpeed,
        0,
        rotationVal,
        false,
        true,
        false);

    SmartDashboard.putNumber("DtoPuN/RmngDist", llyangle);

    toofar = AllianceUtil.isRedAlliance() && FieldConstants.FIELD_LENGTH / 2 > m_swerve.getX()
        || !AllianceUtil.isRedAlliance() && m_swerve.getX() < FieldConstants.FIELD_LENGTH / 2;

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
    return m_transfer.noteAtIntake() || m_intake.noteMissed || toofar
        || RobotBase.isSimulation() && m_transfer.simnoteatintake;
  }
}
