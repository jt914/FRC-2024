// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Gyro {
  // private AHRS m_gyro;
  private Drivetrain drivetrain;
  public static ADIS16470_IMU m_gyro;
  private double gyroAngleOffset = 0;


  /** Creates a new Gyro. */
  public Gyro(double offset) {
    
    /* Create a gyro */
    try {
      m_gyro = new ADIS16470_IMU();
    } 
    catch (RuntimeException ex) {
        System.out.println("--------------");
        System.out.println("NavX not plugged in");
        System.out.println("--------------");
    } 
    gyroAngleOffset = offset;
  }

  /* Returns the total angle (goes past 360) */
  /* ANGLE ADJUSTMENT DOES EFFECT THIS VALUE */
  public double getTotalAngleDegrees() {
    return m_gyro.getAngle() + gyroAngleOffset;
  }

  /* Returns an angle from -180 to 180 */
  /* ANGLE ADJUSTMENT DOES NOT EFFECT THIS VALUE */
  /* Can use gyro.zeroYaw() to offset the value */
  // public double get180AngleDegrees() {
  //   return m_gyro.getYaw();
  // }
  
  /* Resets gyro angle*/
  public void resetGyroYaw() {
    m_gyro.reset();
  }

  /* Calibrates the gyro */
  /* ROBOT MUST BE STILL WHILE CALIBRATING */
  public void calibrateGyro() {
    m_gyro.calibrate();
  }

  // public boolean isCalibrating() {
  //   if (m_gyro.isCalibrating() == true) {
  //     return true;
  //   }
  //   else {
  //     return false;
  //   }
  // }
  
  /* Sets the gyro offset angle */
  /* If the gyro thinks right is forward, then the adjustment should be 90 */
  /* If the gyro thinks left is forward, then the adjustment should be -90 */ 
  /* If the gyro thinks backward is forward, then the adjustment should be 180 */ 
  public void setGyroAngleOffset(double adjustment) {
    gyroAngleOffset = adjustment;
  }

  // public double getVelocityY() {
  //   double velocityY = -m_gyro.getVelocityX();
  //   SmartDashboard.putNumber("Gyro Velocity Y", velocityY);
  //   return velocityY;
  // }

  // public double getVelocityX() {
  //   double velocityX = m_gyro.getVelocityY();
  //   SmartDashboard.putNumber("Gyro Velocity X", velocityX);
  //   return velocityX;
  // }

  // public double getAveragedVelocityY() {
  //   double averagedVelocityY = (getVelocityX() + swerveDrive.chassisYVelocity()) / 2;
  //   SmartDashboard.putNumber("Averaged Velocity Y", averagedVelocityY);
  //   return averagedVelocityY;
  // }

  // public double getAveragedVelocityX() {
  //   double averagedVelocityX = (getVelocityY() - swerveDrive.chassisXVelocity()) / 2;
  //   SmartDashboard.putNumber("Averaged Velocity X", averagedVelocityX);
  //   return averagedVelocityX;
  // }

  /* - - - VELOCITIES RELATIVE TO THE HUB - - - */

  // double gyroStrafe = (remote.getRightY()) * Math.sin(Math.toRadians(-swerveDrive.gyro.getAngle())) - (remote.getRightX()) * Math.cos(Math.toRadians(-swerveDrive.gyro.getAngle()));
  // double gyroForward = (remote.getRightY()) * Math.cos(Math.toRadians(-swerveDrive.gyro.getAngle())) + (remote.getRightX()) * Math.sin(Math.toRadians(-swerveDrive.gyro.getAngle()));
  // public double getHubRelativeVelocityX() {
  //   double hubRelativeVelocityX = (getAveragedVelocityY()) * Math.sin(Math.toRadians(limelight.getXCrosshairOffset())) - getAveragedVelocityX() * Math.cos(Math.toRadians(limelight.getXCrosshairOffset()));
  //   SmartDashboard.putNumber("Hub Relative Velocity X", hubRelativeVelocityX);
  //   return hubRelativeVelocityX;
  // }

  // public double getHubRelativeVelocityY() {
  //   double hubRelativeVelocityY = (getAveragedVelocityY()) * Math.cos(Math.toRadians(limelight.getXCrosshairOffset())) + getAveragedVelocityX() * Math.sin(Math.toRadians(limelight.getXCrosshairOffset()));
  //   SmartDashboard.putNumber("Hub Relative Velocity Y", hubRelativeVelocityY);
  //   return hubRelativeVelocityY;
  // }

  // public double getHubRelativeVelocity() {
  //   double hubRelativeVelocity = Math.sqrt(Math.pow(getHubRelativeVelocityX(), 2) + Math.pow(getHubRelativeVelocityY(), 2));
  //   SmartDashboard.putNumber("Hub Relative Velocity", hubRelativeVelocity);
  //   return hubRelativeVelocity;
  // }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
}
