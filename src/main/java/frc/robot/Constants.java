// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final class DriveConstants {
        public final static int driveMotorID = 1;
        public final static int turnMotorID = 2;
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    }

    public static final class ModuleConstants{
        public static final double kWheelDiamaters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double kTurnMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiamaters;
        public static final double kTurnEncoderRot2Rad = kTurnMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurnEncoderRPM2MeterPerSec = kTurnEncoderRot2Rad / 60;
        public static final double kPTurn = 0.5;
    }
}