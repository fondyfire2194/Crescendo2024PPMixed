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

    public InterpolatingDoubleTreeMap armAngleMap = new InterpolatingDoubleTreeMap();
    /** Shooter look up table key: feet, values: rpm */
    public InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();

    public ShootingData() {
        {
            si.clear();
            
            si.add(new ShotInfo(4.25, 60, 3000));

            si.add(new ShotInfo(5.25, 51, 3000));

            si.add(new ShotInfo(6.25, 46, 3000));

            si.add(new ShotInfo(7.25, 42, 3000));

            si.add(new ShotInfo(8.25, 39, 3000));

            si.add(new ShotInfo(9.25, 36, 3250));

            si.add(new ShotInfo(10.25, 34, 3500));

            si.add(new ShotInfo(11.25, 32, 3500));

            si.add(new ShotInfo(12.25, 30, 3500));

            si.add(new ShotInfo(13.25, 28, 3500));

            si.add(new ShotInfo(14.25, 27, 3750));

            si.add(new ShotInfo(15.25, 26, 4000));

            si.add(new ShotInfo(16.25, 25, 4000));

            si.add(new ShotInfo(17.25, 24, 4250));

            si.add(new ShotInfo(18.25, 23.5, 4500));

            si.add(new ShotInfo(19.25, 22, 4500));

        }

        /** Arm angle look up table key: meters, values: degrees */

        for (int i = 0; i < si.size(); i++) {
            armAngleMap.put(si.get(i).getDistance(),
                    si.get(i).getArm());
        }
        for (int i = 0; i < si.size(); i++) {
            shooterRPMMap.put(si.get(i).getDistance(), si.get(i).getSpeed());
        }
    }

    public class ShotInfo {
        private final double distance;
        private final double speed;
        private final double arm;

        /**
         * Constructs a new ShotInfo.
         * 
         * @param distance of shot in feet
         * @param speed    The speed of the shot, in RPM.
         * @param arm      The angle of the arm, in degrees.
         */
        public ShotInfo(double distance, double arm, double speed) {
            this.distance = Units.feetToMeters(distance);
            this.arm = arm;
            this.speed = speed;
        }

        public double getDistance() {
            return this.distance;
        }

        public double getArm() {
            return this.arm;
        }

        public double getSpeed() {
            return this.speed;
        }

    }
}