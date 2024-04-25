// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Transfer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class TransferIntakeToSensor extends Command {
  private final TransferSubsystem m_transfer;
  private final IntakeSubsystem m_intake;
  private Timer endTimer = new Timer();

  /** Creates a new TransferIntakeToSensor. */
  public TransferIntakeToSensor(TransferSubsystem transfer, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transfer = transfer;
    m_intake = intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTimer.reset();
    endTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_transfer.runToSensor();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transfer.stopMotor();
    if (DriverStation.isTeleopEnabled())
      m_intake.stopMotor();
    m_intake.noNote = endTimer.hasElapsed(3);
    if (!m_intake.noNote)
      m_intake.notesIntaken += 1;
    m_transfer.enableLimitSwitch(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_transfer.noteAtIntake() || endTimer.hasElapsed(3) || RobotBase.isSimulation() && endTimer.hasElapsed(1);
  }
}
