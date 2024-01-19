// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.SwerveCommand;
import frc.robot.Subsystems.Swerve.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {
    Field2d m_field = new Field2d();

  
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.

  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    Constants.m_swerve.updateOdometry();
  }
  
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    CommandScheduler.getInstance().schedule(new SwerveCommand());
    SmartDashboard.putData("Field", m_field);


  }

  boolean fieldRelative = false;
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    m_field.setRobotPose(Constants.m_swerve.m_odometry.getPoseMeters());

  }

  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

}