// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.wpilibj.ADIS16470_IMU;


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
  public void setGyroAngleOffset(double adjustment) {
    gyroAngleOffset = adjustment;
  }


}
