// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Pref;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapTuneGo0 extends TrapezoidProfileCommand {
  /** Creates a new TrapTune. */
  public TrapTuneGo0(SwerveSubsystem swerve) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Pref.getPref("drivemps"), Pref.getPref("drivempsps"))),
        state -> {
          // Use current trajectory state here
          swerve.drive(state.velocity, 0, 0, false, false, false);

          SmartDashboard.putNumber("veltgt", state.velocity);
          // SmartDashboard.putNumber("velact",swerve.getChassisSpeeds().vxMetersPerSecond);

          SmartDashboard.putNumber("disttgt", state.position);
          SmartDashboard.putNumber("distact", swerve.getX());

        },
        // Goal state
        () -> new TrapezoidProfile.State(0, 0),
        // Current state
        TrapezoidProfile.State::new, swerve);
  }
}
