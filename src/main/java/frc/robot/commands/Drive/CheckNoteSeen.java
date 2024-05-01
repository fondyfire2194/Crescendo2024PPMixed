// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LimelightVision;

public class CheckNoteSeen extends Command {
  /** Creates a new AlignToTagSetShootSpeed. */

  private final LimelightVision m_llv;
  private final String m_camname;
  private final double m_timeDelay;
  private double angleError;

  private double distanceApprox;

  public CheckNoteSeen(
      LimelightVision llv,
      String camname,
      double timeDelay) {

    m_llv = llv;
    m_camname = camname;
    m_timeDelay = timeDelay;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_llv.setRearNoteDetectorPipeline();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get Values, Deadband */

    // get horizontal angle

    if (LimelightHelpers.getTV(m_camname)) {

      angleError = LimelightHelpers.getTX(m_camname);

      distanceApprox = LimelightHelpers.getTY(m_camname);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timeDelay > 3.5 || (Math.abs(angleError) < .4 && Math.abs(distanceApprox) < .4);
  }
}
