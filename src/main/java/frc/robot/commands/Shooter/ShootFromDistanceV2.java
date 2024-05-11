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

public class ShootFromDistanceV2 extends Command {
  /** Creates a new JogShooter. */
  private ShooterSubsystem m_shooter;
  private SwerveSubsystem m_swerve;
  private ArmSubsystem m_arm;
  public double distance;
  private double speakerSlotCenterHeight = Units.inchesToMeters(80);
  private double shooterArmPivotPointHeight = Units.inchesToMeters(10);
  private double heightDifference = speakerSlotCenterHeight - shooterArmPivotPointHeight;

  public ShootFromDistanceV2(ShooterSubsystem shooter, SwerveSubsystem swerve, ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_swerve = swerve;
    m_arm = arm;
    // addRequirements(m_shooter, m_arm);
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

    double armAngleRads = calcAngleFromDistance(distance);

    m_arm.setGoal(armAngleRads);
    SmartDashboard.putNumber("DistanceAngle", Units.radiansToDegrees(armAngleRads));

    m_shooter.startShooter(Constants.shooterRPMMap.get(distance));
    SmartDashboard.putNumber("DistanceRPM", m_shooter.topCommandRPM);
    SmartDashboard.putNumber("Distance", distance);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double calcAngleFromDistance(double distance) {
    return Math.atan(heightDifference / distance);
  }
}
