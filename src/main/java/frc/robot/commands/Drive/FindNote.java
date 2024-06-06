// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LLPipelines.pipelines;

public class FindNote extends Command {
  /** Creates a new AlignToTagSetShootSpeed. */

  private final SwerveSubsystem m_swerve;
  private final String m_camname;
  private final double m_maxTravel;

  private double rotationVal;
  private boolean noteInRange;
  private double startDegrees;
  private double endDegrees;
  private double travelLimit;
  private Timer simNoteTimer = new Timer();
  private double angleError;

  public FindNote(
      SwerveSubsystem swerve,
      double maxTravel,
      String camname)

  {
    m_swerve = swerve;
    m_maxTravel = maxTravel;
    m_camname = camname;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    simNoteTimer.reset();
    simNoteTimer.start();
    travelLimit = m_maxTravel;
    if (AllianceUtil.isRedAlliance())
      travelLimit = -travelLimit;
    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname, pipelines.NOTEDETECT1.ordinal());
    startDegrees = m_swerve.getYaw().getDegrees();
    SmartDashboard.putNumber("FindNote/ROTSStart", startDegrees);
    endDegrees = (startDegrees + travelLimit) % 360;
    if (endDegrees > 180)
      endDegrees -= 360;
    if (endDegrees < -180)
      endDegrees += 360;
    SmartDashboard.putNumber("FindNote/ROTSEnd", endDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("FindNote/RotsAct", m_swerve.getYaw().getDegrees());
    /* Get Values, Deadband */

    // get horizontal angle

    boolean noteSeen = LimelightHelpers.getTV(m_camname) || RobotBase.isSimulation() && simNoteTimer.hasElapsed(2);
    double simAngle = m_swerve.getYaw().getDegrees();

    noteInRange = false;

    SmartDashboard.putBoolean("FindNote/NoteSeen", noteSeen);

    if (noteSeen) {

      double distanceApprox = 1;
      if (RobotBase.isReal())
        LimelightHelpers.getTY(m_camname);

      noteInRange = Math.abs(distanceApprox) < 5;
      SmartDashboard.putBoolean("FindNote/InRange", noteInRange);
      if (noteInRange) {
        angleError = LimelightHelpers.getTX(m_camname);

        rotationVal = m_swerve.m_alignNotePID.calculate(angleError, 0);
      }
    } else {
      rotationVal = .25;
      if (travelLimit < 0)
        rotationVal *= -1;
    }
    /* Drive */
    m_swerve.drive(
        0, 0,
        rotationVal *= Constants.SwerveConstants.kmaxAngularVelocity,
        false,
        true,
        false);
    if (RobotBase.isSimulation())
      angleError = 0;
    SmartDashboard.putNumber("FindNote/angleError", angleError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("FindNote/Ended", 1);
    m_swerve.drive(0, 0, 0, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noteInRange && Math.abs(angleError) < 1 || Math.abs(m_swerve.getYaw().getDegrees() - endDegrees) < 1;
  }
}
