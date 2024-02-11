
package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Subsystems.Swerve.Gyro;

public final class Constants {

    /* Controller Constants */
    public static final CommandXboxController swerveController = new CommandXboxController(1);
    public static final CommandXboxController alternateController = swerveController;

    public static boolean intakeRunning = false;
    public static boolean fieldRelative = false;
    public static final double swerveControllerLeftStickDeadband = 0.1;
    public static final double swerveControllerRightXDeadband = 0.1;


    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    public static SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(5);
    public static SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(5);
    public static SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);
    public static double[] randomDesired = new double[]{5,0.1,0.1};

    /* Mechanisms */
    // public static final Gyro m_gyro = new Gyro(90);
    public static final Gyro m_gyro = null;

    public static final Drivetrain m_swerve = null;
    public static final Arm arm = null;
    public static final Intake intake = new Intake();
    public static final Shooter shooter = null;
    public static final Camera camera = new Camera();
    


    /* - - - SWERVE DRIVE CONSTANTS - - - */

    public static final int frontLeftDriveID = 4;
    public static final int frontRightDriveID = 6;
    public static final int backLeftDriveID = 2;
    public static final int backRightDriveID = 7;

    public static final int frontLeftTurnID = 3;
    public static final int frontRightTurnID = 5;
    public static final int backLeftTurnID = 1;
    public static final int backRightTurnID = 8;

    public static final double frontLeftOffset = 0;
    public static final double frontRightOffset = 0;
    public static final double backLeftOffset = 0;
    public static final double backRightoffset = 0;

    public static final double driveEncoderVelocityConversion = (1/5.33);
    public static final double driveEncoderPositionConversion = 1;
    public static final double turnEncoderPositionConversion = 1;

    public static final double drivetrainModuleOffset = 0.2923; /* Assuming the robot is square, the X & Y offset from the center of rotation to each module */
    public static final int numberOfModules = 4;
    public static final double maxVelocityMultiplier = 6; /* Max velocity in m/s */
    public static final double radiansPerSecondMultiplier = 6; /* Max angular rate in radians/second */

    public static final double armClimbOffset = 1;

    /* - - - OTHER CONSTANTS - - - */

    // public static final int intakeID = 13; 
    // public static final int shooterTopID = 11; 
    // public static final int shooterBotID = 12; 
    // public static final int armLeftID = 10; 
    // public static final int armRightID = 9; 

    public static final int intakeID = 10; 
    public static final int shooterTopID = 10; 
    public static final int shooterBotID = 10; 
    public static final int armLeftID = 10; 
    public static final int armRightID = 10; 


}