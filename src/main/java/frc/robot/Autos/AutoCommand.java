package frc.robot.Autos;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drivetrain;
import edu.wpi.first.math.geometry.Transform3d;

public class AutoCommand extends Command {
    // private Arm arm;
    // private Intake intake;
    private boolean isFinished = false;
    // private Shooter shooter;
    private Drivetrain swerve;
    private int swerveStep;
    private int shooterStep;
    private int intakeStep;
    private int armStep;
    private Transform3d target;
    private int timer;
    private int step = 1;
    private PIDController aimController = new PIDController(.23, 0.000001, 0);
    private PIDController drivePID = new PIDController(2, 0, 0);


    public AutoCommand(){
        swerve = Constants.swerve;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("timer", timer);


        //test step by step. Make sure step 1 works by commenting out step = 2, then move on. 


        //step 1 = Ramp up shooter, auto aim yaw
        if(step == 1){
            System.out.println("step 1 now");

            //when to move on to the next step
            if(timer >= 200){
                timer = 0;
                step = 2;
            }
            timer++;


            double[] desired = Constants.camera.getDesiredShoot(0);
            if(desired != null && desired[0] != 0){
            //   Constants.arm.setDesired(desired[1] * 3.29 + 14.3);
            //   Constants.shooter.setVelocity();
              Constants.swerve.drive(0,0,aimController.calculate(desired[0], 0));
            }

        }

        else if(step == 2){
        System.out.println("step 2 now");

            //wait 50 cycles of code for shooting, then move on.
            if(timer >= 200){
                timer = 0;
                step = 3;
                // Constants.arm.setDesired(4.89);
                Constants.intake.run();
            }
            timer++;
            // Constants.intake.runFast();
        }

        //intake should be running, this is driving to the note

        //maybe add an xSpeed * 
        else if(step == 3){
            System.out.println("step 3 now");

            if(Constants.intake.hasNote()){
                Constants.intake.stop();
                step = 4;
            }
            else{
                target = Constants.camera.getTarget();
                SmartDashboard.putNumber("xCurrent", target.getX());
                SmartDashboard.putNumber("yCurrent", target.getY());
                double xSpeed = drivePID.calculate(target.getX(), 1.44);
                double ySpeed = drivePID.calculate(target.getY(), 1.77);
                SmartDashboard.putNumber("xCalculated", xSpeed);
                SmartDashboard.putNumber("yCalculated", ySpeed);

                double yaw = 0;

                double[] desired = Constants.camera.getDesiredShoot(ySpeed);
                if(desired != null && desired[0] != 0){
                    yaw = aimController.calculate(desired[0], 0);
                }
                Constants.swerve.drive(xSpeed, ySpeed, yaw);
            }
        }

        //ramping up the shooter and autoaiming yaw for the first note
        else if (step == 4){

            if(timer >= 200){
                timer = 0;
                step = 5;
                System.out.println("step 5 now");
            }
            timer++;


            double[] desired = Constants.camera.getDesiredShoot(0);
            if(desired != null && desired[0] != 0){
            //   Constants.arm.setDesired(desired[1] * 3.29 + 14.3);
            //   Constants.shooter.setVelocity();
              Constants.swerve.drive(0,0,aimController.calculate(desired[0], 0));

            }

        }

        //shooting, after 50 cycles it puts the intake back down
        else if(step == 5){

            if(timer >= 200){
                timer = 0;
                step = 6;
                // Constants.arm.setDesired(4.89);
                // Constants.intake.run();
            }
            timer++;
            // Constants.intake.runFast();

        }
        

        //driving to the next note
        else if(step == 6){
            if(Constants.intake.hasNote()){
                step = 7;
            }
            else{
                target = Constants.camera.getTarget();
                double xSpeed = drivePID.calculate(target.getX(), 0);
                double ySpeed = drivePID.calculate(target.getY(), 1.6);
                double yaw = 0;
                double[] desired = Constants.camera.getDesiredShoot(ySpeed);
                if(desired != null && desired[0] != 0){
                    yaw = aimController.calculate(desired[0], 0);
                }
                Constants.swerve.drive(xSpeed, ySpeed, yaw);
            }


        }
        
        //shooting the next note, autoaiming
        else if(step == 7){
            
            if(timer >= 50){
                timer = 0;
                step = 8;
                System.out.println("step 8 now");
            }
            timer++;


            double[] desired = Constants.camera.getDesiredShoot(0);
            if(desired != null && desired[0] != 0){
            //   Constants.arm.setDesired(desired[1] * 3.29 + 14.3);
            //   Constants.shooter.setVelocity();
              Constants.swerve.drive(0,0,aimController.calculate(desired[0], 0));

            }

        }

        //shooting, then putting intake back down.
        else if(step == 8){
            
            if(timer >= 50){
                timer = 0;
                step = 9;
                Constants.arm.setDesired(4.89);
                Constants.intake.run();
            }
            timer++;
            Constants.intake.runFast();

        }






        
    }

    @Override
    public void end(boolean interrupted) {

    }
}

