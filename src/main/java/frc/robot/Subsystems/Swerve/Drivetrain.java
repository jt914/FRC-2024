// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxVelocity = 2; // meters/second (m/s) //cannot go over 3.3
  public static final double kMaxVoltage = kMaxVelocity / (((473 / 9.25) * 0.103 * Math.PI) / 60); /* THIS CANNOT GO OVER 12 VOLTS */
  public static final double kMaxAngularSpeed = 2 * Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation;
  private final Translation2d m_frontRightLocation;
  private final Translation2d m_backLeftLocation;
  private final Translation2d m_backRightLocation;

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  public final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;
  private ChassisSpeeds speeds;

  public SwerveModuleState[] states = new SwerveModuleState[4];

  private final SwerveDriveKinematics m_kinematics;
  public final SwerveDrivePoseEstimator poseEstimator;

  public double xSpeed, ySpeed;

  public final PIDController drivePIDController = new PIDController(1.2, 0.001, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  public final PIDController turnPIDController = new PIDController(0.006, 0.000, 0.00001);

  public final SimpleMotorFeedforward driveSimpleMotorFeedforward = new SimpleMotorFeedforward(.00001, 0);
  public Gyro m_gyro;

  public Drivetrain() {
    m_gyro = Constants.m_gyro;
    m_gyro.resetGyroYaw();

    m_frontLeftLocation = new Translation2d(Constants.drivetrainModuleOffset, Constants.drivetrainModuleOffset);
    m_frontRightLocation = new Translation2d(Constants.drivetrainModuleOffset, -Constants.drivetrainModuleOffset);
    m_backLeftLocation = new Translation2d(-Constants.drivetrainModuleOffset, Constants.drivetrainModuleOffset);
    m_backRightLocation = new Translation2d(-Constants.drivetrainModuleOffset, -Constants.drivetrainModuleOffset);

    m_frontLeft = new SwerveModule(Constants.frontLeftDriveID, Constants.frontLeftTurnID);
    m_frontRight = new SwerveModule(Constants.frontRightDriveID, Constants.frontRightTurnID);
    m_backLeft = new SwerveModule(Constants.backLeftDriveID, Constants.backLeftTurnID);
    m_backRight = new SwerveModule(Constants.backRightDriveID, Constants.backRightTurnID);

    m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    poseEstimator = 
      new SwerveDrivePoseEstimator(
        m_kinematics, 
        Rotation2d.fromDegrees(m_gyro.getTotalAngleDegrees()), 
        new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition() },
        // new Pose2d(new Translation2d(14.700758, 8.2042), new Rotation2d(3* Math.PI / 4)));
        new Pose2d());

    //     AutoBuilder.configureHolonomic(
    //     this::getPose, // Robot pose supplier
    //     this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //     this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //     this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //     new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //         new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //         new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    //         4.5, // Max module speed, in m/s
    //         0.4, // Drive base radius in meters. Distance from robot center to furthest module.
    //         new ReplanningConfig() // Default path replanning config. See the API for the options here
    //     ),
    //     new BooleanSupplier() {
        
    //     },
    //     this // Reference to this subsystem to set requirements
    // );
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return speeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds){

    
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);
    // states = swerveModuleStates;

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 12);
    m_frontLeft.setModuleState(swerveModuleStates[0], 0);
    m_frontRight.setModuleState(swerveModuleStates[1], 1);
    m_backLeft.setModuleState(swerveModuleStates[2], 2);
    m_backRight.setModuleState(swerveModuleStates[3], 3);

    // SmartDashboard.putNumber("xSpeed", xSpeed);
    // SmartDashboard.putNumber("ySpeed", ySpeed);
    // SmartDashboard.putNumber("rotation", yaw);

  }



  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose){

    poseEstimator.resetPosition(
          new Rotation2d(Constants.m_gyro.getTotalAngleDegrees()),
          new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()}, 
          pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double yaw) {

    speeds = new ChassisSpeeds(xSpeed, ySpeed, yaw);
    SmartDashboard.putNumber("xSpeedSPEED", xSpeed);
        SmartDashboard.putNumber("ySpeedSPEED", ySpeed);

    // var swerveModuleStates = m_kinematics.toSwerveModuleStates(
    //         fieldRelative
    //             ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
    //             : new ChassisSpeeds(xSpeed, ySpeed, rot));

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, yaw));
    // System.out.println(Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)));
    
    Logger.recordOutput("MyStates", swerveModuleStates);
    // states = swerveModuleStates;

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 12);
    m_frontLeft.setModuleState(swerveModuleStates[0], 0);
    m_frontRight.setModuleState(swerveModuleStates[1], 1);
    m_backLeft.setModuleState(swerveModuleStates[2], 2);
    m_backRight.setModuleState(swerveModuleStates[3], 3);


    
    // SmartDashboard.putNumber("xSpeed", xSpeed);
    // SmartDashboard.putNumber("ySpeed", ySpeed);
    // SmartDashboard.putNumber("rotation", yaw);
  }

  public double getSpeed(){
    return 0;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {

        // Compute the robot's field-relative position exclusively from vision measurements.

        // try{
        //   EstimatedRobotPose visionMeasurement3d = Constants.camera.getPose().get();
        //   Pose2d visionMeasurement2d = visionMeasurement3d.estimatedPose.toPose2d();
        //   poseEstimator.addVisionMeasurement(visionMeasurement2d,visionMeasurement3d.timestampSeconds);
        // }catch (Exception e){
        // }

        // System.out.println(Constants.camera.getLatestResult().getBestTarget().getFiducialId());



  
    // Convert robot pose from Pose3d to Pose2d needed to apply vision measurements.
    
    // SwerveDriveWheelPositions wheelPositions = new SwerveDriveWheelPositions(new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()});
    // Rotation2d angle = new Rotation2d(Constants.m_gyro.getTotalAngleDegrees());

    // Twist2d twist = m_kinematics.toTwist2d(m_previousWheelPositions, wheelPositions);
    // twist.dtheta = angle.minus(m_previousAngle).getRadians();
    // SmartDashboard.putNumber("twist", twist.dtheta);
    // SmartDashboard.putNumber("twistX", twist.dx);
    // SmartDashboard.putNumber("twistY", twist.dy);

    // SmartDashboard.putString("previous", m_previousWheelPositions.toString());
    // SmartDashboard.putString("current", wheelPositions.toString());
    //CHECK IF ROTATING CORRECT AMT
    poseEstimator.update(
        Rotation2d.fromDegrees(m_gyro.getTotalAngleDegrees()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()


        });

    SmartDashboard.putNumber("frontLeft", m_frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("frontRight", m_frontRight.getPosition().distanceMeters);

  }

  public void resetModules() {
    m_frontLeft.setTurnEncoder(0);
    m_frontRight.setTurnEncoder(0);
    m_backLeft.setTurnEncoder(0);
    m_backRight.setTurnEncoder(0);
  }
}