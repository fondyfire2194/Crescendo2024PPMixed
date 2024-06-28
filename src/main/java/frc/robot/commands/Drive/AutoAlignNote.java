// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Pref;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LimelightHelpers;

public class AutoAlignNote extends Command {

  private final SwerveSubsystem m_swerve;
  private final boolean m_endAtTargets;
  private final double m_toleranceDegrees;

  public PIDController m_alignTargetPID = new PIDController(0.008, 0, 0);

  private double rotationVal;
  private Timer elapsedTime;
  String rearCamName = CameraConstants.rearCamera.camname;
  private boolean visionTargetSet;
  private double lastAngleError;
  private int loopctr;
  private double setpoint;

  public AutoAlignNote(
      SwerveSubsystem swerve, double toleranceDegrees, boolean endAtTargets) {

    m_swerve = swerve;
    m_endAtTargets = endAtTargets;
    m_toleranceDegrees = toleranceDegrees;

    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_alignTargetPID.enableContinuousInput(-180, 180);
    m_alignTargetPID.setTolerance(m_toleranceDegrees);
    m_alignTargetPID.setIZone(1);
    m_alignTargetPID.setIntegratorRange(-.01, .01);
    m_alignTargetPID.setI(.000);
    m_alignTargetPID.reset();
    m_swerve.targetPose = AllianceUtil.flipFieldAngle(FieldConstants.centerNotes[m_swerve.targetNote]);
    elapsedTime = new Timer();
    elapsedTime.reset();
    elapsedTime.start();
    visionTargetSet = false;
    loopctr = 0;
    setpoint = Pref.getPref("autoalignoffset");
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopctr++;
    if (LimelightHelpers.getTV(rearCamName)) {
      double angleError = LimelightHelpers.getTX(rearCamName);
      if (loopctr > 10 && Math.abs(angleError - lastAngleError) < 1) {
        visionTargetSet = true;

        rotationVal = m_alignTargetPID.calculate(angleError, setpoint);
      }
      lastAngleError = angleError;
    }

    m_swerve.alignedToTarget=m_alignTargetPID.atSetpoint();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_alignTargetPID.reset();
    m_swerve.drive(0, 0, 0, false, true, false);
    SmartDashboard.putString("Aligned", "to Note");
    SmartDashboard.putBoolean("AlignedVTS", visionTargetSet);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_endAtTargets && (elapsedTime.hasElapsed(.25) && m_swerve.alignedToTarget || elapsedTime.hasElapsed(5)
        || RobotBase.isSimulation());
  }
}
