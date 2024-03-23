// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.*;
import frc.robot.Autos.*;
// import frc.robot.Autos.AutoCommand;
import frc.robot.Commands.*;
import frc.robot.Commands.Climber.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Command OneNote  = new OneNote();
  private final Command OneNoteDrive  = new OneNoteDrive();
  private final Command twoRight  = new TwoNoteCommand();



  // The robot's subsystems and commands are defined here...
  //

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureButtonBindings();
    m_chooser.setDefaultOption("One Note", OneNote);
    m_chooser.addOption("One Note Drive", OneNoteDrive);
    m_chooser.addOption("Two Note", twoRight);
    SmartDashboard.putData(m_chooser);


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    (Constants.swerveController.start()).onTrue(new FlashCommand());
    (Constants.swerveController.start()).onTrue(new SwerveCommand());
    (Constants.swerveController.b()).onTrue(new IntakeShootCommand());
    // Constants.alternateController.x().onTrue(new WinchCommand());

    (Constants.swerveController.rightBumper()).toggleOnTrue(new IntakeCommand());
    (Constants.swerveController.x()).toggleOnTrue(new ResetModulesCommand());
    (Constants.swerveController.leftBumper()).whileTrue(new IntakeReverseCommand());
    (Constants.swerveController.a()).toggleOnTrue(new AmpCommand());
    Constants.swerveController.y().onTrue(new ToggleAutoAimCommand());

    Constants.alternateController.rightTrigger().whileTrue(new RightOutClimber());
    Constants.alternateController.rightBumper().whileTrue(new RightInClimber());

    Constants.alternateController.leftTrigger().whileTrue(new LeftOutClimber());
    Constants.alternateController.leftBumper().whileTrue(new LeftInClimber());


    //Press x to turn the arm on. If something goes wrong, just press X again and it will turn the arm off

  } 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}