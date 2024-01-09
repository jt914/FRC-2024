// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

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

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 8.0; // Max Voltage, Max Velocity = (kMaxSpeed * motor kv * (1/(drive gear ratio)) * (0.106 * pi))
  public static final double kMaxAngularSpeed = 2 * Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.drivetrainModuleOffset, Constants.drivetrainModuleOffset);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.drivetrainModuleOffset, -Constants.drivetrainModuleOffset);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.drivetrainModuleOffset, Constants.drivetrainModuleOffset);
  private final Translation2d m_backRightLocation = new Translation2d(-Constants.drivetrainModuleOffset, -Constants.drivetrainModuleOffset);

  // private final SwerveModule m_frontLeft = new SwerveModule(Constants.frontLeftDriveID, Constants.frontLeftTurnID);
  // private final SwerveModule m_frontRight = new SwerveModule(Constants.frontRightDriveID, Constants.frontRightTurnID);
  // private final SwerveModule m_backLeft = new SwerveModule(Constants.backLeftDriveID, Constants.backLeftTurnID);
  // private final SwerveModule m_backRight = new SwerveModule(Constants.backRightDriveID, Constants.backRightTurnID);

   final SwerveModule m_frontLeft = new SwerveModule(Constants.frontRightDriveID, Constants.frontRightTurnID);
   final SwerveModule m_frontRight = new SwerveModule(Constants.frontLeftDriveID, Constants.frontLeftTurnID);
   final SwerveModule m_backLeft = new SwerveModule(Constants.backRightDriveID, Constants.backRightTurnID);
   final SwerveModule m_backRight = new SwerveModule(Constants.backLeftDriveID, Constants.backLeftTurnID);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);


  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
    
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double yaw, boolean fieldRelative) {

    // var swerveModuleStates = m_kinematics.toSwerveModuleStates(
    //         fieldRelative
    //             ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
    //             : new ChassisSpeeds(xSpeed, ySpeed, rot));

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, yaw));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setModuleState(swerveModuleStates[0], 0);
    m_frontRight.setModuleState(swerveModuleStates[1], 1);
    m_backLeft.setModuleState(swerveModuleStates[2], 2);
    m_backRight.setModuleState(swerveModuleStates[3], 3);
    // SmartDashboard.putNumber("xSpeed", xSpeed);
    // SmartDashboard.putNumber("ySpeed", ySpeed);
    // SmartDashboard.putNumber("rotation", yaw);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
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