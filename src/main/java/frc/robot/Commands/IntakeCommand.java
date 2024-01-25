package frc.robot.Commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;

public class IntakeCommand extends Command {
    private Arm arm;
    private Intake intake;
    private boolean isFinished = false;




    @Override
    public void initialize(){
        arm = Constants.arm;
        intake = Constants.intake;
        Constants.arm.setAngle(10);

        
    }

    @Override
    public void execute(){
        Constants.arm.setAngle(10);
        intake.run();
    }
    @Override
    public boolean isFinished(){
        return isFinished;

    }

    @Override
    public void end(boolean interrupted) {
    // Constants.intakeStatus = false;
        intake.stop();
        Constants.arm.stop();
        NetworkTableInstance.getDefault().getTable("/datatable").getEntry("IntakeCommand").setBoolean(false);

    }





    
}


