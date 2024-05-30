// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ShootingData {

    public ArrayList<ShotInfo> si = new ArrayList<ShotInfo>();

    public InterpolatingDoubleTreeMap armToleranceMap = new InterpolatingDoubleTreeMap();

    public InterpolatingDoubleTreeMap armAngleMap = new InterpolatingDoubleTreeMap();
    /** Shooter look up table key: feet, values: rpm */
    public InterpolatingDoubleTreeMap shotTimeMap = new InterpolatingDoubleTreeMap();

    public InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();

    public ShootingData() {
        {
            si.clear();
            // distance feet, angle degrees, speed rpm, time ms, tolerance deg
            // returns meters, radians, rpm, seconds, radians
            si.add(new ShotInfo(4.25, 60, 3000, 300, 2));

            si.add(new ShotInfo(5.25, 51, 3000, 300, 2));

            si.add(new ShotInfo(6.25, 46, 3000, 300, 2));

            si.add(new ShotInfo(7.25, 42, 3000, 300, 2));

            si.add(new ShotInfo(8.25, 39, 3000, 300, 2));

            si.add(new ShotInfo(9.25, 36, 3250, 300, 2));

            si.add(new ShotInfo(10.25, 34, 3500, 300, 2));

            si.add(new ShotInfo(11.25, 32, 3500, 300, 2));

            si.add(new ShotInfo(12.25, 30, 3500, 300, 2));

            si.add(new ShotInfo(13.25, 28, 3500, 300, 2));

            si.add(new ShotInfo(14.25, 27, 3750, 300, 2));

            si.add(new ShotInfo(15.25, 26, 4000, 300, 2));

            si.add(new ShotInfo(16.25, 25, 4000, 300, 2));

            si.add(new ShotInfo(17.25, 24, 4250, 300, 2));

            si.add(new ShotInfo(18.25, 23.5, 4500, 300, 2));

            si.add(new ShotInfo(19.25, 22, 4500, 300, 2));

        }

        /** Arm angle look up table key: meters, values: degrees */

        for (int i = 0; i < si.size(); i++) {
            armAngleMap.put(si.get(i).getDistanceMeters(),
                    si.get(i).getArmRads());
        }
        for (int i = 0; i < si.size(); i++) {
            armToleranceMap.put(si.get(i).getDistanceMeters(),
                    si.get(i).getToleranceRads());
        }
        for (int i = 0; i < si.size(); i++) {
            shooterRPMMap.put(si.get(i).getDistanceMeters(), si.get(i).getSpeedRPM());
        }
        for (int i = 0; i < si.size(); i++) {
            shotTimeMap.put(si.get(i).getDistanceMeters(), si.get(i).getTimeSec());
        }

    }

    public class ShotInfo {
        private final double distanceFeet;
        private final double speedRPM;
        private final double armDegrees;
        private final double timeMs;
        private final double toleranceDegrees;

        /**
         * Constructs a new ShotInfo.
         * 
         * @param distance  of shot in feet
         * @param speed     The speed of the shot, in RPM.
         * @param arm       The angle of the arm, in degrees.
         * @param timems    Time from shooter to speaker
         * @param tolerance Arm tolerance in degrees
         */
        public ShotInfo(double distanceFeet, double armDegrees, double speedRPM, double timeMs,
                double toleranceDegrees) {
            this.distanceFeet = distanceFeet;
            this.armDegrees = armDegrees;
            this.speedRPM = speedRPM;
            this.timeMs = timeMs;
            this.toleranceDegrees = toleranceDegrees;
        }

        public double getDistanceMeters() {
            return Units.feetToMeters(distanceFeet);
        }

        public double getArmRads() {
            return Units.degreesToRadians(armDegrees);
        }

        public double getSpeedRPM() {
            return this.speedRPM;
        }

        public double getToleranceRads() {
            return Units.degreesToRadians(toleranceDegrees);
        }

        public double getTimeSec() {
            return timeMs / 1000;
        }

    }
}