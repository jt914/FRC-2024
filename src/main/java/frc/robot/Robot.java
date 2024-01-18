// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.SwerveCommand;

public class Robot extends TimedRobot {
  


  @Override
  public void robotInit() {
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }


  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    // Constants.m_swerve.updateOdometry();
  }
  
  public void teleopInit() {
    CommandScheduler.getInstance().schedule(new SwerveCommand());


    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();

  }

  @Override
  public void teleopPeriodic() {
    // Constants.m_swerve.updateOdometry();
  }

  public void disabledInit() {
  }

}