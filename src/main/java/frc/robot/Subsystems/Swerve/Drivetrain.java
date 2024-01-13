// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Robot;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxVelocity = 0.35; // meters/second (m/s)
  public static final double kMaxVoltage = kMaxVelocity / (((473 / 9.25) * 0.103 * Math.PI) / 60); /* THIS CANNOT GO OVER 12 VOLTS */
  public static final double kMaxAngularSpeed = 2 * Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation;

  private final SwerveModule m_frontLeft, m_frontRight, m_backLeft, m_backRight;

  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDriveOdometry m_odometry;

  private Gyro m_gyro;

  public Drivetrain() {
    m_gyro = Constants.m_gyro;
    m_gyro.resetGyroYaw();

    m_frontLeftLocation = new Translation2d(Constants.drivetrainModuleOffset, -Constants.drivetrainModuleOffset);
    m_frontRightLocation = new Translation2d(Constants.drivetrainModuleOffset, Constants.drivetrainModuleOffset);
    m_backLeftLocation = new Translation2d(-Constants.drivetrainModuleOffset, -Constants.drivetrainModuleOffset);
    m_backRightLocation = new Translation2d(-Constants.drivetrainModuleOffset, Constants.drivetrainModuleOffset);

    m_frontLeft = new SwerveModule(Constants.frontRightDriveID, Constants.frontRightTurnID);
    m_frontRight = new SwerveModule(Constants.frontLeftDriveID, Constants.frontLeftTurnID);
    m_backLeft = new SwerveModule(Constants.backRightDriveID, Constants.backRightTurnID);
    m_backRight = new SwerveModule(Constants.backLeftDriveID, Constants.backLeftTurnID);

    m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);


    m_odometry = 
      new SwerveDriveOdometry(
        m_kinematics, 
        Rotation2d.fromDegrees(m_gyro.getTotalAngleDegrees()), 
        new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition() }
        );
    
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

    // var swerveModuleStates = m_kinematics.toSwerveModuleStates(
    //         fieldRelative
    //             ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
    //             : new ChassisSpeeds(xSpeed, ySpeed, rot));

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, yaw));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 12);
    m_frontLeft.setModuleState(swerveModuleStates[0], 0);
    m_frontRight.setModuleState(swerveModuleStates[1], 1);
    m_backLeft.setModuleState(swerveModuleStates[2], 2);
    m_backRight.setModuleState(swerveModuleStates[3], 3);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getTotalAngleDegrees()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public void resetModules() {
    m_frontLeft.setTurnEncoder(0);
    m_frontRight.setTurnEncoder(0);
    m_backLeft.setTurnEncoder(0);
    m_backRight.setTurnEncoder(0);
  }
}