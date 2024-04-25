// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GeometryUtil;
import frc.robot.utils.LLPipelines;

public class LimelightSetStartPose extends Command {
  /** Creates a new LimelightSetStartPose. */

  private final String m_llName;
  private final SwerveSubsystem m_swerve;
  private final Pose2d m_pathStartPose;
  private boolean redAlliance;
  private boolean blueAlliance;
  private Pose2d useAsStartPose;
  private double startTime;
  private double allowedTime = .75;
  private boolean endIt;
  private int loopctr;
  private int validResults = 3;

  public LimelightSetStartPose(String llName, SwerveSubsystem swerve, Pose2d pathStartPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_llName = llName;
    m_swerve = swerve;
    m_pathStartPose = pathStartPose;
    startTime = Timer.getFPGATimestamp();
    endIt = false;
    loopctr = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    redAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;
    blueAlliance = alliance.isPresent() && alliance.get() == Alliance.Blue;

   // LimelightHelpers.setPipelineIndex(m_llName, LLPipelines.pipelines.APRILTAGSTART.ordinal());

    useAsStartPose = m_pathStartPose;

    if (redAlliance)

      useAsStartPose = GeometryUtil.flipFieldPose(m_pathStartPose);

    m_swerve.setPose(useAsStartPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean llhastarget = LimelightHelpers.getTV(m_llName);

    if (llhastarget) {
      loopctr++;
      Pose2d botPose2d = LimelightHelpers.getBotPose2d(m_llName);
      useAsStartPose = botPose2d;
    }

    else {
      loopctr = 0;

    }

    if (loopctr > validResults && Timer.getFPGATimestamp() > startTime + allowedTime) {
      // m_swerve.setPose(useAsStartPose);
      endIt = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endIt;
  }
}
