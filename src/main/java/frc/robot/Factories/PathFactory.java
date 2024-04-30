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
import monologue.Logged;

/** Add your docs here. */
public class PathFactory implements Logged{

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
        A_CN1ToCN2,
        A_CN1ToPU,
        A_CN1ToShoot,
        A_CN2ToCN1,
        A_CN2ToPU,
        A_CN2ToShoot,
        A_CN3ToPU,
        A_N1ToCN1,
        A_N1ToCN2,
        A_S2N1,
        A_SToCN3;
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

    public void linkAmpPaths() {
        pathMaps.clear();
        for (amppaths a : amppaths.values()) {
            pathMaps.put(a.name(), getPath(a.name()));
        }
    }

    public enum sourcepaths {
        SourceToCenter4,
        SourceToCenter4ShootMoving,
        SourceToCenter5,
        SourceToNearCenter4,
        SourceToNearCenter5,
        SourceToNearCenter4_5,
        NearCenter4toCenter4,
        SourceShootToCenter4,
        SourceShootToCenter5,
        Center4ToSourceShoot,
        Center5ToSourceShoot,
        Center4ToCenter5,
        Center5ToCenter4;
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

    // public enum decisionpoints {
    // CN1,
    // CN2, CN3,
    // CN4,
    // CN5;
    // }

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