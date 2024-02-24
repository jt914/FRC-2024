package frc.robot.Commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Lights;

public class IntakeCommand extends Command {
    private Arm arm;
    private Intake intake;
    private boolean isFinished = false;
    public int elapsed = 0;
    public boolean triggered = false;
    private Lights lights;

    @Override
    public void initialize(){
        intake = Constants.intake;
        triggered = false;
        elapsed = 0;
        arm = Constants.arm;
        arm.setDesired(4.89);
        addRequirements(Constants.arm);
        lights = Constants.lights;

    }

    @Override
    public void execute(){
        intake.run();
        lights.setColor(0, 75, 255, 255, 50);
        SmartDashboard.putNumber("IntakeSens", Constants.intake.intakeSensor.getVoltage());
        SmartDashboard.putBoolean("Intake Trigger", triggered);
        if(Constants.intake.intakeSensor.getVoltage()<.5) {
            triggered = true;
        }
        if(triggered == true) {
            elapsed++;
        }
        if(elapsed > 3) {
            isFinished = true;
            elapsed = 0;
            lights.setColor(0, 75, 56, 255, 50);
        }
        arm.moveArm();
 
    }
    @Override
    public boolean isFinished(){
         
        return isFinished;

    }

    @Override
    public void end(boolean interrupted) {
        
        Constants.intake.stop();
        isFinished = false;
        if(interrupted) {
            lights.setColor(0, 75, 0, 0, 0);
        }
    }
 
}

