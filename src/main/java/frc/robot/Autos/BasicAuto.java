package frc.robot.Autos;
import java.util.function.BooleanSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ResetModulesCommand;
import frc.robot.Commands.ToggleAutoAimCommand;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class BaseAuto extends Command {
    private boolean isFinished = false;
    private Drivetrain swerve = Constants.swerve;
    private int step = -1;
    private Trigger trig;
    private InterpolatingDoubleTreeMap tm = new InterpolatingDoubleTreeMap();

    public BaseAuto(){
        autoAim = () -> false;
        trig = new Trigger(autoAim);
        trig.onTrue(new ToggleAutoAimCommand());
    }
  
    @Override
    public void initialize() {
        tm.put(3.74, 33.0);
        tm.put(2.4, 26.0);
        tm.put(1.3, 15.5);
    }
  
    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
    }

}
