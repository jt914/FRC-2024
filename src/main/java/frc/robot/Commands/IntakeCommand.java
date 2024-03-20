package frc.robot.Commands;
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
    public Lights lights;
    public IntakeCommand(){
        addRequirements(Constants.intake);

    }
    @Override
    public void initialize(){
        intake = Constants.intake;
        triggered = false;
        elapsed = 0;
        arm = Constants.arm;
        arm.setDesired(5.5);
        addRequirements(Constants.arm);
        lights = Constants.lights;
    }   

    @Override
    public void execute(){
        intake.run();
        lights.setColorRed(30, 150, 50);

        if(Constants.intake.intakeSensor.getVoltage()<.5) {
            triggered = true;
        }
        if(triggered == true) {
            elapsed++;
        }
        if(elapsed > 2) {
            isFinished = true;
            elapsed = 0;
            Constants.hasNote = true;
            lights.setColorGreen(30, 150, 50);
        }
    }
    @Override
    public boolean isFinished(){
         
        return isFinished;

    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            lights.off();
        }
        Constants.intake.stop();
        isFinished = false;
    }
}

