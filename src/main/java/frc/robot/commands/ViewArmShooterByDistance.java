// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Factories.CommandFactory;

public class ViewArmShooterByDistance extends Command {
  /** Creates a new ArmShooterByDistance. */
  private final CommandFactory m_cf;
  double distance;
  int loopctr;

  public ViewArmShooterByDistance(CommandFactory cf) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cf = cf;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = 1;
    loopctr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loopctr++;
    if (loopctr == 5) {
      double rpm = Constants.shooterRPMMap.get(distance);
      double angle = Constants.armAngleMap.get(distance);
      double calcAngle = m_cf.getArmAngleFromTarget(FieldConstants.speakerSlotHeight, distance);
      double stageAngle = m_cf.getArmAngleFromTarget(FieldConstants.stageHeight, distance);

      double angleTan = Math.tan(Units.degreesToRadians(angle));
      double angleCalcTan = Math.tan(Units.degreesToRadians(calcAngle));

      double aimHeightFromTable = angleTan * distance;
      double aimHeightFromCalc = angleCalcTan * distance + ArmConstants.armPivotZ;

      SmartDashboard.putNumber("ArmCalc//DistRPM", rpm);
      SmartDashboard.putNumber("ArmCalc//DistAngle", angle);
      SmartDashboard.putNumber("ArmCalc//CalcAngle", calcAngle);
      SmartDashboard.putNumber("ArmCalc//StageAngle", stageAngle);
      SmartDashboard.putNumber("ArmCalc//CalcAngleTan", angleTan);
      SmartDashboard.putNumber("ArmCalc//TableAngleTan", angleCalcTan);

      SmartDashboard.putNumber("ArmCalc//AimHeightTable", aimHeightFromTable);
      SmartDashboard.putNumber("ArmCalc//AimHeightCalc", aimHeightFromCalc);

      SmartDashboard.putNumber("ArmCalc//DistDist", distance);
      loopctr = 0;
      distance += .1;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance >= 7;// Units.feetToMeters(19.25);
  }
}
