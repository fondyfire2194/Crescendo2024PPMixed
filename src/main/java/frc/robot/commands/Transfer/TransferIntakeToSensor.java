// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Transfer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class TransferIntakeToSensor extends Command {
  private final TransferSubsystem m_transfer;
  private final IntakeSubsystem m_intake;
  private final double m_noNoteTime;
  private Timer endTimer = new Timer();
  private double simmotetime = 3.5;

  /** Creates a new TransferIntakeToSensor. */
  public TransferIntakeToSensor(TransferSubsystem transfer, IntakeSubsystem intake, double noNotetime) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transfer = transfer;
    m_intake = intake;
    m_noNoteTime = noNotetime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTimer.reset();
    endTimer.start();
    m_intake.noteMissed = false;
    m_transfer.simnoteatintake = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_transfer.runToSensor();
    m_intake.noteMissed = RobotBase.isSimulation() && (m_transfer.skipFirstNoteInSim || m_transfer.skipSecondNoteInSim)
        || endTimer.hasElapsed(m_noNoteTime);
    m_transfer.simnoteatintake = RobotBase.isSimulation() && !m_transfer.skipSecondNoteInSim
        && endTimer.hasElapsed(simmotetime)
        && !m_transfer.skipFirstNoteInSim && !m_transfer.skipSecondNoteInSim;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transfer.stopMotor();
    if (DriverStation.isTeleopEnabled())
      m_intake.stopMotor();
    m_transfer.enableLimitSwitch(false);
    m_transfer.skipFirstNoteInSim = false;
    m_transfer.skipSecondNoteInSim = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_transfer.noteAtIntake() || m_intake.noteMissed || RobotBase.isSimulation() && m_transfer.simnoteatintake;
  }
}
