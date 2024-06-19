// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import java.util.HashMap;
import java.io.File;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class PathFactory {

    private final SwerveSubsystem m_swerve;

    int ampChoice;
    int ampChoiceLast;

    int sourceChoice;
    int sourceChoiceLast;

    public boolean ampFilesOK;

    public boolean sourceFilesOK;

    public HashMap<String, PathPlannerPath> pathMaps = new HashMap<String, PathPlannerPath>();

    public PathFactory(SwerveSubsystem swerve) {
        m_swerve = swerve;
    }

    public enum amppaths {
        AmpShootToCenter1,
        AmpShootToCenter2,
        AmpShootToCenter3,        
        AmpToCenter1,
        AmpToCenter2,
        AmpToWing1,
        Center1ToCenter2,
        Center2toCenter1,
        Center1ToAmpShoot,
        Center2ToAmpShoot,
        Center3ToWing2,
        Center3ToAmpShoot;
    }

    public boolean checkAmpFilesExist() {
        int valid = 0;
        for (amppaths a : amppaths.values()) {
            if (new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + a.name() + ".path").isFile())
                valid++;
            SmartDashboard.putNumber("Valid", valid);
        }
        return valid == amppaths.values().length;
    }

    public boolean linkAmpPaths() {
        pathMaps.clear();
        for (amppaths a : amppaths.values()) {
            pathMaps.put(a.name(), getPath(a.name()));
        }
        return true;
    }

    public enum sourcepaths {
        SourceToCenter4,
        SourceToCenter5,
        SourceShootToCenter3,
        SourceShootToCenter4,
        SourceShootToCenter5,
        Center4ToCenter5,
        Center5ToCenter4,
        Center4ToSourceShoot,
        Center5ToSourceShoot;
    }

    public boolean checkSourceFilesExist() {
        int valid = 0;
        for (sourcepaths a : sourcepaths.values()) {
            if (new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + a.toString() + ".path").isFile())
                valid++;
        }
        return valid == sourcepaths.values().length;
    }

    public void linkSourcePaths() {
        pathMaps.clear();
        for (sourcepaths s : sourcepaths.values()) {
            pathMaps.put(s.toString(), getPath(s.toString()));
        }
    }

    public enum sbwfrpaths {
        SubwfrShootToWing1,
        SubwfrShootToWing2,
        SubwfrShootToWing3,
        SubwfrShootToWing1Shoot,
        SubwfrShootToWing3Shoot,
        SubwfrShootToCenter2,
        Wing1ToSubwfrShoot,
        Wing2ToSubwfrShoot,
        Wing3ToSubwfrShoot,
        Wing2ToCenter3,
        Center3ToSubwfrShoot,
        QuickToNote3,
        Quick3ToNote1,
        Center3ToWing2
    }

    public boolean checkSbwfrFilesExist() {
        int valid = 0;
        for (sbwfrpaths a : sbwfrpaths.values()) {
            if (new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + a.toString() + ".path").isFile())
                valid++;
        }
        return valid == sbwfrpaths.values().length;
    }

    public void linkSbwfrPaths() {
        pathMaps.clear();
        for (sbwfrpaths s : sbwfrpaths.values()) {
            pathMaps.put(s.toString(), getPath(s.toString()));
        }
    }

    public PathPlannerPath getPath(String pathname) {
        return PathPlannerPath.fromPathFile(pathname);
    }

    public Command setStartPosebyAlliance(PathPlannerPath path) {
        Pose2d temp = path.getPreviewStartingHolonomicPose();
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return Commands.runOnce(() -> m_swerve.resetPoseEstimator(flipPose(temp)));
        } else
            return Commands.runOnce(() -> m_swerve.resetPoseEstimator(temp));
    }

    public static Pose2d flipPose(Pose2d pose) {
        return GeometryUtil.flipFieldPose(pose);
    }

}