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
  public static final double kMaxVelocity = 1.8; // meters/second (m/s) //cannot go over 3.3
  public static final double kMaxVoltage = kMaxVelocity / (((Constants.RPMperVolt / Constants.driveGearRatio) * Constants.wheelDiameter * Math.PI) / 60); /* THIS CANNOT GO OVER 12 VOLTS */
  public static final double kMaxAngularSpeed = 3 * Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation;
  private final Translation2d m_frontRightLocation;
  private final Translation2d m_backLeftLocation;
  private final Translation2d m_backRightLocation;

  public final SwerveModule m_frontLeft;
  public final SwerveModule m_frontRight;
  public final SwerveModule m_backLeft;
  public final SwerveModule m_backRight;
  private ChassisSpeeds speeds;

  public SwerveModuleState[] states = new SwerveModuleState[4];

  private final SwerveDriveKinematics m_kinematics;
  public final SwerveDrivePoseEstimator poseEstimator;

  public double xSpeed, ySpeed;

  public final PIDController drivePIDController = new PIDController(1.2, 0.001, 0);

  public final PIDController turnPIDController = new PIDController(0.006, 0.000, 0.00001);

  public PIDController odometryController = new PIDController(1.8, .1, 0);

  public final SimpleMotorFeedforward driveSimpleMotorFeedforward = new SimpleMotorFeedforward(.00001, 0);
  public Gyro m_gyro;

  public Drivetrain() {
    m_gyro = Constants.m_gyro;
    m_gyro.resetGyroYaw();

    m_frontLeftLocation = new Translation2d(Constants.drivetrainModuleOffset, Constants.drivetrainModuleOffset);
    m_frontRightLocation = new Translation2d(Constants.drivetrainModuleOffset, -Constants.drivetrainModuleOffset);
    m_backLeftLocation = new Translation2d(-Constants.drivetrainModuleOffset, Constants.drivetrainModuleOffset);
    m_backRightLocation = new Translation2d(-Constants.drivetrainModuleOffset, -Constants.drivetrainModuleOffset);

    m_frontLeft = new SwerveModule(Constants.frontLeftDriveID, Constants.frontLeftTurnID, Constants.frontLeftCANCoderID);
    m_frontRight = new SwerveModule(Constants.frontRightDriveID, Constants.frontRightTurnID, Constants.frontRightCANCoderID);
    m_backLeft = new SwerveModule(Constants.backLeftDriveID, Constants.backLeftTurnID, Constants.backLeftCANCoderID);
    m_backRight = new SwerveModule(Constants.backRightDriveID, Constants.backRightTurnID, Constants.backRightCANCoderID);

    m_frontLeft.setMagnetOffset(-0.020751953125);
    m_frontRight.setMagnetOffset(-0.041748046875);
    m_backLeft.setMagnetOffset(0.268310546875);
    m_backRight.setMagnetOffset(0.24755859375);

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
        new Pose2d());

  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return speeds;
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


    var swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, yaw));

    

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 12);
    m_frontLeft.setModuleState(swerveModuleStates[0], 0);
    m_frontRight.setModuleState(swerveModuleStates[1], 1);
    m_backLeft.setModuleState(swerveModuleStates[2], 2);
    m_backRight.setModuleState(swerveModuleStates[3],3);

  }
  /**
   * Returns next position increment to reach xMeters
   * @param xMeters Desired position in meters for x axis(Forward+Back)
   */
  public double tunedDriveX(double xMeters) {
    return odometryController.calculate(poseEstimator.getEstimatedPosition().getX(), xMeters);
  }
  /**
   * Returns next position increment to reach yMeters
   * @param yMeters Desired position in meters for y axis(Left+Right)
   */
  public double tunedDriveY(double yMeters) {
    return odometryController.calculate(poseEstimator.getEstimatedPosition().getY(), yMeters);
  }

  public double getSpeed(){
    return 0;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {


    poseEstimator.update(
        Rotation2d.fromDegrees(m_gyro.getTotalAngleDegrees()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()

        });


  }

  public void resetAllAbsoluteModules() {
    m_frontLeft.resetAbsoluteModule();
    m_frontRight.resetAbsoluteModule();
    m_backLeft.resetAbsoluteModule();
    m_backRight.resetAbsoluteModule();
  }
}