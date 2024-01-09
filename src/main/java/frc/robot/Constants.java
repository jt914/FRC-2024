// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /* - - - SWERVE DRIVE CONSTANTS - - - */
    public static final int frontLeftDriveID = 4;
    public static final int frontRightDriveID = 6;
    public static final int backLeftDriveID = 1;
    public static final int backRightDriveID = 7;

    public static final int frontLeftTurnID = 3;
    public static final int frontRightTurnID = 5;
    public static final int backLeftTurnID = 2;
    public static final int backRightTurnID = 8;


    public static final double driveEncoderPositionConversion = 1;
    public static final double turnEncoderPositionConversion = 1;

    public static final double drivetrainModuleOffset = 0.2923; /* Assuming the robot is square, the X & Y offset from the center of rotation to each module */
    

    /* - - - OTHER CONSTANTS - - - */
    public static final int swerveControllerPort = 0;
    public static final int alternateControllerPort = 1;

    public static final double controllerLeftXDeadband = 0.1;
    public static final double controllerLeftYDeadband = 0.1;
    public static final double controllerRightXDeadband = 0.1;

    public static final double xSlewRateLimiter = 5;
    public static final double ySlewRateLimiter = 5;
    public static final double yawSlewRateLimiter = 5;

    // /* - - - FLYWHEEL CONSTANTS - - - */
    // public static final int leftFlywheelID = 4;
    // public static final int rightFlywheelID = 5;
    // public static final double RPMtoFlywheelTipSpeed = (6 * Math.PI) / 2362; /* Multiply by the RPM to get the tip speed of a 6in flywheel */

    // /* - - - HOOD CONSTANTS - - - */
    // public static final int hoodMotorID = 11;
    // public static final double motorToHoodConversion = 360 * (16/32) * (10/203);
    // public static final double distanceToHoodAngle = 1/3; /* Temporary number for testing, real "equation" will use data points and a line of best fit */

    // /* - - - INDEXER CONSTANTS - - - */
    // public static final int outerIndexerID = 15;
    // public static final int innerIndexerID = 12;
    // public static final double outerIndexerSpeed = 0; /* Max outer indexer speed from 0 to 1 */
    // public static final double innerIndexerSpeed = 0; /* Max inner indexer speed from 0 to 1 */
    
    // /* - - - INTAKE CONSTANTS - - - */
    // public static final int intakeMotorID = 16;
    // public static final int pneumaticsHubID = 0;
    // public static final int solenoidOne = 14;
    // public static final int solenoidTwo = 15;
    // public static final double intakeSpeed = 0.4; /* Max intake speed from 0 to 1 */

    // /* - - - CLIMBER CONSTANTS - - - */
    // public static final int rightClimberID = 9;
    // public static final int leftClimberID = 13;
    // public static final double climberSpeed = 0.4; /* Max climber speed from 0 to 1 */

    // /* - - - LIMELIGHT CONSTANTS - - - */
    // public static final double limelightHeight = 0.5 /*meters*/;
    // public static final double hubHeight = 2.64 /*meters*/;
    // public static final double limelightAngle = 25 /*degrees*/;
}
