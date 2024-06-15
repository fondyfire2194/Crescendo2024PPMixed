// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;


import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;

public class CheckOKSwitchToDrive extends Command {
  /**
   * Switch to drive to note using vision after a remaining distance to field
   * center line
   * and only if a note is seen.
   * Otherwise let paralle path run out and try to pickup note
   */
  private final SwerveSubsystem m_swerve;

  private final CommandFactory m_cf;

  private final double m_switchoverDistance;

  public CheckOKSwitchToDrive(
      SwerveSubsystem swerve,
      CommandFactory cf,
      double switchOverDistance) {
    m_swerve = swerve;
    m_cf = cf;
    m_switchoverDistance = switchOverDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.noteSeen = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get Values, Deadband */

    // get horizontal angle

    m_cf.doIntake(10, 5);

    if (AllianceUtil.isRedAlliance())
      m_swerve.remainingdistance = m_swerve.getX() - FieldConstants.FIELD_LENGTH / 2;
    else
      m_swerve.remainingdistance = FieldConstants.FIELD_LENGTH / 2 - m_swerve.getX();

    SmartDashboard.putNumber("RMGDST", m_swerve.remainingdistance);

    m_swerve.noteSeen = RobotBase.isReal() && LimelightHelpers.getTV(CameraConstants.rearCamera.camname)
        && m_swerve.remainingdistance <= m_switchoverDistance
        || RobotBase.isSimulation() && m_swerve.remainingdistance < 1.5;

    SmartDashboard.putBoolean("RMGns", m_swerve.noteSeen);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_swerve.noteSeen;
  }
}
