// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;
import javax.swing.text.StyleContext.SmallAttributeSet;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
  // private AHRS m_gyro;
  private Drivetrain drivetrain;
  public static AHRS m_gyro;
  private double gyroAngleOffset = 0;

  /** Creates a new Gyro. */
  public Gyro(double offset) {
    
    /* Create a gyro */
    try {
      m_gyro = new AHRS();
    } 
    catch (RuntimeException ex) {
        System.out.println("--------------");
        System.out.println("NavX not plugged in");
        System.out.println("--------------");
    } 
    gyroAngleOffset = offset;
  }

  public double getTotalAngleDegrees() {
    double angle = m_gyro.getAngle() + gyroAngleOffset;

    return angle;
  }

  
  public void resetGyroYaw() {
    m_gyro.reset();
  }

  public void calibrateGyro() {
    m_gyro.reset();
  }

  public void setGyroAngleOffset(double adjustment) {
    gyroAngleOffset = adjustment;
  }
}