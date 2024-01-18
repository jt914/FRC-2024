// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
  private static final int kEncoderResolution = 1;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  public RelativeEncoder driveEncoder;
  public RelativeEncoder turnEncoder;

  private final PIDController turnPIDController = new PIDController(0.006, 0.000, 0.00001);
      

  private double turnEncoder180;
  
  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   */
  public SwerveModule(int driveMotorID, int turnMotorID) 
  {
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnMotorID , MotorType.kBrushless);

    driveMotor.setIdleMode(IdleMode.kCoast);
    turnMotor.setIdleMode(IdleMode.kCoast);


    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();

    driveEncoder.setPosition(0);
    turnEncoder.setPosition(0);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    driveEncoder.setPositionConversionFactor(Constants.driveEncoderPositionConversion);
    turnEncoder.setPositionConversionFactor(Constants.turnEncoderPositionConversion);

    

    
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turnPIDController.enableContinuousInput(-180,  180);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setModuleState(SwerveModuleState desiredState, int module) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    double moduleVelocity = desiredState.speedMetersPerSecond;
    double moduleAngle = desiredState.angle.getDegrees();
    double optimizedModuleOutput[] = glacierOptimized(moduleAngle, getTurn180Angle(), moduleVelocity);

    double driveOutput = optimizedModuleOutput[1];
    driveMotor.setVoltage(driveOutput);
      // SmartDashboard.putNumber("driveOutput" + module, driveOutput);

    double turnOutput = MathUtil.clamp(turnPIDController.calculate(getTurn180Angle(), optimizedModuleOutput[0]), -0.4, 0.4);
    turnMotor.set(-turnOutput);
      // SmartDashboard.putNumber("turnOutput " + module, turnOutput);
      // SmartDashboard.putNumber("getTurn180Angle " + module, getTurn180Angle());

  }

  /**
   * 
   * @param desiredModuleAngle Angle passed into the module to move to
   * @param currentModuleAngle Current angle of the module
   * @param moduleVelocity In voltage (relies on motor kv and gear ratio)
   * @return Optimized angle and velocity calculations, in that order
   */
  public double[] glacierOptimized(double desiredModuleAngle, double currentModuleAngle, double moduleVelocity) {
    double optimized[] = new double[2];
      if (Math.abs(desiredModuleAngle - currentModuleAngle) > 90 && Math.abs(desiredModuleAngle) + Math.abs(currentModuleAngle) < 270) {
        if (desiredModuleAngle >= 0) {
            optimized[0] = desiredModuleAngle - 180;
            optimized[1] = moduleVelocity * -1;
        }
        else if (desiredModuleAngle <= 0){
          optimized[0] = desiredModuleAngle + 180;
          optimized[1] = moduleVelocity * -1;
        }
      }
      else {
        optimized[0] = desiredModuleAngle;
        optimized[1] = moduleVelocity;
      }
    return optimized;
  }
  
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()*23.684));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition()/8, new Rotation2d(turnEncoder.getPosition()/15.1975));

  }

  /**
   * Converts the motor's encoder from an absolute angle to a limited range between -180 to 180
   * 
   * @return Angle of the module converted to a range between -180 degrees and 180 degrees
   */
  public double getTurn180Angle() {
    if (turnEncoder.getPosition()*23.684 > 360) {
      turnEncoder180 = ((turnEncoder.getPosition()*23.684) % 360) - 180;
    }
    else if (turnEncoder.getPosition()*23.684 < 0) {
      turnEncoder180 = ((turnEncoder.getPosition()*23.684) % 360) + 180;
    }
    else {
      turnEncoder180 = (turnEncoder.getPosition()*23.684) - 180;
    }
    return turnEncoder180;
  }

  public void setTurnEncoder(double angle) {
    turnEncoder.setPosition(angle);
  }
}