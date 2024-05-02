// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;

public class CheckOKSwitchToDrive extends Command {
  /**
   * Switch to drive to note using vision after a remaining distance to field
   * center line
   * and only if a note is seen.
   * Otherwise let paralle path run out and try to pickup note
   */
  private final SwerveSubsystem m_swerve;
  private final LimelightVision m_llv;
  private final double m_switchoverDistance;

  private boolean noteSeen;
  private boolean redAlliance;
  private double remainingdistance;

  public CheckOKSwitchToDrive(
      SwerveSubsystem swerve,
      LimelightVision llv,
      double switchOverDistance) {
    m_swerve = swerve;
    m_llv = llv;
    m_switchoverDistance = switchOverDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_llv.setRearNoteDetectorPipeline();

    redAlliance = DriverStation.getAlliance().isPresent() &&
        DriverStation.getAlliance().get() == Alliance.Red;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get Values, Deadband */

    // get horizontal angle

    noteSeen = LimelightHelpers.getTV(CameraConstants.rearCamera.camname);

    if (redAlliance)
      remainingdistance = m_swerve.getX() - FieldConstants.FIELD_LENGTH / 2;
    else
      remainingdistance = FieldConstants.FIELD_LENGTH / 2 - m_swerve.getX();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return remainingdistance <= m_switchoverDistance && (RobotBase.isSimulation() || noteSeen);
  }
}
