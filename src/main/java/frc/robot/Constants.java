// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
/** Add your docs here. */
public class Constants {

    public static final double ksVolts = 0.90431;
    public static final double kvVoltSecondsPerMeter = 2.3534;
    public static final double kaVoltSecondsSquaredPerMeter = 1.4246;
    public static final double kPDriveVel = 0.00045296;

    public static final double kTrackWidthMeters = 0.52; //HOrizontal distance between wheels

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;


 

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kGearRatio = 1; //12.6
    public static final double kWheelRadiusInches = 3; //3in
    

    public static final double kConvertionFactor =  (1.0 / 2048) * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) / kGearRatio;


   

}