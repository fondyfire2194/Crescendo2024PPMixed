// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.SwerveSubsystem;

public class FindNote extends Command {
  /** Creates a new AlignToTagSetShootSpeed. */

  private final SwerveSubsystem m_swerve;
  private final LimelightVision m_llv;
  private final String m_camname;
  private final boolean m_directionCW;

  private double rotationVal;
  private int angleError;
  private boolean noteInRange;
  private double startAngle;

  public FindNote(
      SwerveSubsystem swerve,
      boolean directionCW,
      LimelightVision llv,
      String camname)

  {
    m_swerve = swerve;
    m_llv = llv;
    m_directionCW = directionCW;
    m_camname = camname;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_llv.setRearNoteDetectorPipeline();
    startAngle = m_swerve.getPose().getRotation().getDegrees();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get Values, Deadband */

    // get horizontal angle

    boolean noteSeen = LimelightHelpers.getTV(m_camname);
    angleError = 10;
    noteInRange = false;

    if (noteSeen) {

      double distanceApprox = LimelightHelpers.getTY(m_camname);

      noteInRange = Math.abs(distanceApprox) < 5;

      if (noteInRange) {

        double angleError = LimelightHelpers.getTX(m_camname);

        rotationVal = m_swerve.m_alignNotePID.calculate(angleError, 0);

      }

    }

    else
      rotationVal = .25;
    if (!m_directionCW)
      rotationVal -= 0;
    /* Drive */
    m_swerve.drive(
        0, 0,
        rotationVal *= Constants.SwerveConstants.kmaxAngularVelocity,
        false,
        true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noteInRange && Math.abs(angleError) < 1;
  }
}
