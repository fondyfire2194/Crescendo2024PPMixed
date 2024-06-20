// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.CameraConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LLPipelines.pipelines;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToFindNote extends PIDCommand {
  /** Creates a new RotateToAngle. */

  private static PIDController rotatePID = new PIDController(0.06, 0, 0);
  private static double startangle;

  public RotateToFindNote(SwerveSubsystem drive, double maxangle) {

    super(
        // The controller that the command will use
        rotatePID,
        // This should return the measurement
        () -> drive.getPose().getRotation().getDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> (startangle + maxangle),
        // This uses the output
        output -> {
          drive.drive(0, 0, output, false, false, false);
        }, drive);

    m_controller.enableContinuousInput(-180, 180);
    // this number could be changed

    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    // getController().setP(Pref.getPref("rotkp"));
    // m_controller.setI(Pref.getPref("rotki"));
    // m_controller.setD(Pref.getPref("rotkd"));

    getController().setP(.01);
    m_controller.setI(.005);
    m_controller.setD(1e-5);

    m_controller.setTolerance(1);
    startangle = drive.getPose().getRotation().getDegrees();
    LimelightHelpers.setPipelineIndex(
        CameraConstants.rearCamera.camname, pipelines.NOTEDET1.ordinal());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.m_controller.atSetpoint() || LimelightHelpers.getTV(CameraConstants.rearCamera.camname);
  }
}
