// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.opencv.photo.MergeDebevec;

import edu.wpi.first.math.VecBuilder;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class LimelightTagsUpdate {

    private final String m_camname;
    private final SwerveSubsystem m_swerve;
    private final boolean m_useMegaTag2;
    boolean rejectUpdate;

    public LimelightTagsUpdate(String camname, SwerveSubsystem swerve, boolean useMegaTag2) {

        m_camname = camname;
        m_swerve = swerve;
        m_useMegaTag2 = useMegaTag2;
    }

    public void execute() {

        if (m_useMegaTag2) {

            LimelightHelpers.SetRobotOrientation(m_camname,
                    m_swerve.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(),
                    0, 0, 0, 0, 0);

            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_camname);

            rejectUpdate = mt2.tagCount == 0 || Math.abs(m_swerve.getGyroRate()) > 720;

            if (!rejectUpdate) {
                m_swerve.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                m_swerve.getPoseEstimator().addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
            }
        }

        else {

            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_camname);

            rejectUpdate = mt1.tagCount == 0
                    || mt1.tagCount == 1 && mt1.rawFiducials.length == 1 && mt1.rawFiducials[0].ambiguity > .7;

            if (!rejectUpdate) {
                m_swerve.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                m_swerve.getPoseEstimator().addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);
            }
        }
    }

}
