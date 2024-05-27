// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Factories.CommandFactory;
import frc.robot.subsystems.ArmSubsystem;

public class ViewArmShooterByDistance extends Command {
  /** Creates a new ArmShooterByDistance. */
  private final CommandFactory m_cf;
  private final ShootingData m_sd;
  private final ArmSubsystem m_arm;
  double distance;
  int loopctr;
  boolean endit;
  double deg;
  double distcomp = 0;

  public ViewArmShooterByDistance(CommandFactory cf, ShootingData sd, ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cf = cf;
    m_sd = sd;
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = 1;
    loopctr = 0;
    endit = false;
    deg = 60;
    distcomp = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loopctr++;
    if (loopctr == 5 && distance <= 7) {
      double rpm = m_sd.shooterRPMMap.get(distance);
      double angleDeg = Units.radiansToDegrees(m_sd.armAngleMap.get(distance));
      double shotTimeMs = m_sd.shotTimeMap.get(distance) * 1000;
      double toleranceDeg = Units.radiansToDegrees(m_sd.armToleranceMap.get(distance));

      double calcAngle = m_sd.armCalcDistMap.get(distance);

      double stageAngle = m_cf.getLobArmAngleFromTarget(distance);

      double angleTan = Math.tan(Units.degreesToRadians(angleDeg));
      double angleCalcTan = Math.tan(Units.degreesToRadians(calcAngle));

      double aimHeightFromTable = angleTan * distance;

      SmartDashboard.putNumber("ArmCalc/DistRPM", rpm);
      SmartDashboard.putNumber("ArmCalc/DistAngle", angleDeg);
      SmartDashboard.putNumber("ArmCalc/ShotTime", shotTimeMs);
      SmartDashboard.putNumber("ArmCalc/ToleranceAngle", toleranceDeg);

      SmartDashboard.putNumber("ArmCalc/CalcAngle", calcAngle);

      SmartDashboard.putNumber("ArmCalc/StageAngle", stageAngle);

      SmartDashboard.putNumber("ArmCalc/AimHeightTable", aimHeightFromTable);

      SmartDashboard.putNumber("ArmCalc/DistDist", distance);
      loopctr = 0;
      distance += .1;
    }

    endit = distance >= 7;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endit;// Units.feetToMeters(19.25);
  }
}
