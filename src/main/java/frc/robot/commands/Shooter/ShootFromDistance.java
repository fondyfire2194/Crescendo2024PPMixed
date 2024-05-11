// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootFromDistance extends Command {
  /** Creates a new JogShooter. */
  private ShooterSubsystem m_shooter;
  private SwerveSubsystem m_swerve;
  private ArmSubsystem m_arm;
  public double distance;

  public ShootFromDistance(ShooterSubsystem shooter, SwerveSubsystem swerve, ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_swerve = swerve;
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setTolerance(ArmConstants.angleTolerance);
    m_arm.resetController();
    m_arm.enable();
  }

  @Override
  public void execute() {
    distance = m_swerve.getDistanceFromSpeaker();
    m_arm.setGoal(Units.degreesToRadians(Constants.armAngleMap.get(distance)));
    SmartDashboard.putNumber("DistanceAngle", Constants.armAngleMap.get(distance));

    m_shooter.startShooter(Constants.shooterRPMMap.get(distance));
    SmartDashboard.putNumber("DistanceRPM", m_shooter.topCommandRPM);
    SmartDashboard.putNumber("Distance", distance);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
