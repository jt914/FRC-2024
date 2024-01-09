// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.util.sendable.*;

public class Robot extends TimedRobot {
  private final Joystick m_controller = new Joystick(Constants.swerveControllerPort);
  private final Drivetrain m_swerve = new Drivetrain();
  int routine1 = 1;
  int routine2 = 2;
  Autos currentRoutine = new Autos();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Constants.xSlewRateLimiter);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Constants.ySlewRateLimiter);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.yawSlewRateLimiter);
  SendableChooser<Integer> choose = new SendableChooser<>();

  boolean started = false;
  @Override
  public void robotInit() {
    started = false;
    m_swerve.resetModules();
    choose.setDefaultOption("Routine 1", routine1);
    choose.addOption("Routine 2", routine2);
    SmartDashboard.putData("Auto: ", choose);




  }
  
  
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();

  }
  
  @Override
  public void autonomousPeriodic() {
    m_swerve.updateOdometry();
  }
  public void autonomousInit()
  {
    currentRoutine.runAuto(choose.getSelected());

  }

  @Override
  public void teleopPeriodic() {
    if (m_controller.getRawButtonPressed(1) == true) {
      m_swerve.resetModules();
    }
    if (m_controller.getRawButtonPressed(2) == true) {
      started = true;
    }
    if (started == true) {
      driveWithJoystick(false);
    }
    if(m_controller.getRawButtonPressed(8) == true)
    {

    }
  }

  public void disabledInit() {
    started = false;
  }

  /* DO NOT ATTEMPT TO CHANGE SWERVE CODE; INVERTING A SINGLE VALUE SCREWS WITH EVERYTHING (IDK HOW) */
  /* Wanna change the x and y axes? Or invert the one of the drive directions? Good luck!!! (pain) */
  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward. 
    final var xSpeed =
        m_xspeedLimiter.calculate(MathUtil.applyDeadband(-m_controller.getRawAxis(1), Constants.controllerLeftXDeadband))
            * Drivetrain.kMaxSpeed;
            SmartDashboard.putNumber("xSpeed ", xSpeed);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default. 
    final var ySpeed =
        m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(0), Constants.controllerLeftYDeadband))
            * Drivetrain.kMaxSpeed;
            SmartDashboard.putNumber("ySpeed ", ySpeed);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var yaw =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(2), Constants.controllerRightXDeadband))
            * Drivetrain.kMaxAngularSpeed;
            SmartDashboard.putNumber("yaw ", yaw);

    m_swerve.drive(-xSpeed, -ySpeed, yaw, fieldRelative);
  }
}