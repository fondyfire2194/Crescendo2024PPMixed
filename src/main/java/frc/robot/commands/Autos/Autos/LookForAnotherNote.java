// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Autos;

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
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LLPipelines.pipelines;

public class LookForAnotherNote extends Command {
  private final SwerveSubsystem m_swerve;
  private final TransferSubsystem m_transfer;
  private final IntakeSubsystem m_intake;

  double angleError = 0;
  private Timer elapsedTime = new Timer();
  double startPosition;
  private double distBeyondMidField = .01;
  private boolean trylimit;
  private String camname = CameraConstants.rearCamera.camname;

  public LookForAnotherNote(
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

    LimelightHelpers.setPipelineIndex(camname, pipelines.NOTEDET1.ordinal());

    elapsedTime.reset();
    elapsedTime.start();

    SmartDashboard.putNumber("DtoPuN/swposnX", m_swerve.getX());
    SmartDashboard.putNumber("DtoPuN/swposnY", m_swerve.getY());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotBase.isReal() && LimelightHelpers.getTV(camname)) {
      angleError = LimelightHelpers.getTX(camname);
      SmartDashboard.putNumber("DtoPuN/AngErr", angleError);
    } else
      angleError = 0;

    double rotationVal = m_swerve.m_alignNotePID.calculate(angleError, 0);

    double pickupspeed = -SwerveConstants.notePickupSpeed;

    m_swerve.drive(
        pickupspeed,
        0,
        rotationVal,
        false,
        true,
        false);

    trylimit = m_swerve.ampActive && m_swerve.getY() < (FieldConstants.FIELD_WIDTH / 2 - distBeyondMidField)

        || m_swerve.sourceActive && m_swerve.getY() > (FieldConstants.FIELD_WIDTH / 2 + distBeyondMidField);

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
    return m_transfer.noteAtIntake() || trylimit;

  }
}
