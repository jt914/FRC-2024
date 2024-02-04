package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

public class ShooterCommand extends Command{
    /*
     * actions needed:
     * 1st button, should run 'aim' command, then run shooter. Maybe use synchronized commands to speed up execution    
     * 
     * 2nd button, should move shooter position to source intake
     * 
     * 3rd button, if needed, moves shooter to lowered position for moving under the stage
     */

    private boolean finish;



    @Override
    public void initialize(){
        System.out.println("working");

    }

    
    @Override
    public void execute(){
        Constants.shooter.setSpeed(-0.6, -0.7);

        //idea is that the arm and the shooter motors can continuously adjust incrementally 
        //until they reach within a certain threshold
        
        // double[] optimal = Constants.camera.getDesiredShoot();
        // double desiredAngle = Constants.camera.getDriveOffset();

        //margin of errors, should be adjusted
        // SmartDashboard.putNumber("arm", optimal[0] - Constants.arm.updateAngle());
        // // SmartDashboard.putNumber("gyro", Constants.m_gyro.getTotalAngleDegrees()%360 - desiredAngle);

        // if(finish){
        //     Constants.intake.run(); //to shoot the note out
        //     Constants.shooter.setSpeed(optimal[1], optimal[2]);

        // }
        // else{
        //     // if(Math.abs(optimal[0] - Constants.arm.updateAngle()) < 1 && Math.abs(Constants.m_gyro.getTotalAngleDegrees()%360 - desiredAngle) < 5){
        //     finish = true;

        // else{
        //     Constants.arm.setAngle(optimal[0]);
        //     Constants.shooter.setSpeed(optimal[1], optimal[2]);
        // }

        



        // Constants.m_swerve.drive(0,0, desiredAngle);
        
    }

    
    @Override
    public void end(boolean interrupted){
        Constants.shooter.stop();
        
    }


    @Override
    public boolean isFinished() {
        return false;
    }




    
}
