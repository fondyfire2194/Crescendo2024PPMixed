// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SubwfrStart;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.PathFactory;
import frc.robot.Factories.PathFactory.sbwfrpaths;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class AutoSbwfrShootThenSequence extends SequentialCommandGroup {

        public AutoSbwfrShootThenSequence(
                        CommandFactory cf,
                        PathFactory pf,
                        SubwooferAutoCommands sac,
                        SwerveSubsystem swerve,
                        sbwfrpaths path1,
                        sbwfrpaths path2,
                        sbwfrpaths path3,
                        sbwfrpaths path4) {

                addCommands(

                                sac.setsbwrstart(swerve, cf),

                                sac.sbwfrShoot(cf),

                                sac.moveAndPickup(path1, swerve, cf, pf),

                                sac.sbwfrmoveandshoot(path2, swerve, cf, pf),

                                sac.moveAndPickup(path3, swerve, cf, pf),

                                sac.sbwfrmoveandshoot(path4, swerve, cf, pf),

                                cf.resetAll());

        }

        public AutoSbwfrShootThenSequence(
                        CommandFactory cf,
                        PathFactory pf,
                        SubwooferAutoCommands sac,
                        SwerveSubsystem swerve, 
                        sbwfrpaths path1,
                        sbwfrpaths path2,
                        sbwfrpaths path3,
                        sbwfrpaths path4,
                        sbwfrpaths path5,
                        sbwfrpaths path6) {

                addCommands(
                                sac.setsbwrstart(swerve, cf),

                                sac.sbwfrShoot(cf),

                                sac.moveAndPickup(path1, swerve, cf, pf),

                                sac.sbwfrmoveandshoot(path2, swerve, cf, pf),

                                sac.moveAndPickup(path3, swerve, cf, pf),

                                sac.sbwfrmoveandshoot(path4, swerve, cf, pf),

                                sac.moveAndPickup(path5, swerve, cf, pf),

                                sac.sbwfrmoveandshoot(path6, swerve, cf, pf),

                                cf.resetAll());

        }

}
