// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class DriveToPickupNote extends Command {
  /** Creates a new AlignToTagSetShootSpeed. */

  private final SwerveSubsystem m_swerve;
  private final TransferSubsystem m_transfer;
  private final IntakeSubsystem m_intake;
  private final String m_camname;
  private final LimelightVision m_llv;
  private final Pose2d m_notePose;

  double angleError = 0;
  private Timer elapsedTime = new Timer();
  double endPosition;
  private double yerror;

  public DriveToPickupNote(
      SwerveSubsystem swerve,
      TransferSubsystem transfer,
      IntakeSubsystem intake,
      String camname,
      LimelightVision llv,
      Pose2d notePose)
  {
    m_swerve = swerve;
    m_transfer = transfer;
    m_intake = intake;
    m_camname = camname;
    m_llv = llv;
    m_notePose = notePose;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    elapsedTime.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotBase.isReal() && LimelightHelpers.getTV(m_camname)) {

      angleError = LimelightHelpers.getTX(m_camname);

      SmartDashboard.putNumber("NOTETX", angleError);
    } else
      angleError = 0;
    double rotationVal = m_swerve.m_alignNotePID.calculate(angleError, 0);

     yerror = m_notePose.getY() - m_swerve.getY();
    double xerror = m_notePose.getX() - m_swerve.getX();

    /*
     * Drive
     * assumes robot is pointing in y plane to pickup a center note;
     * could be a higher or lower y value note
     */
    m_swerve.drive(
        SwerveConstants.notePickupSpeed * yerror, // Constants.SwerveConstants.kmaxSpeed *3,
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
    m_transfer.simnoteatintake = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(yerror) < .1 || m_transfer.noteAtIntake();
  }
}
