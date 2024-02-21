// package frc.robot.Autos;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Subsystems.Arm;
// import frc.robot.Subsystems.Intake;
// import frc.robot.Subsystems.Shooter;
// import frc.robot.Subsystems.Swerve.Drivetrain;

// public class FiveNoteAuto extends Command {
//     // private Arm arm;
//     // private Intake intake;
//     private boolean isFinished = false;
//     // private Shooter shooter;
//     private Drivetrain swerve;
//     private int swerveStep;
//     private int shooterStep;
//     private int intakeStep;
//     private int armStep;
//     private long shooterTimer;

//     public FiveNoteAuto(){
//         swerveStep = 0;
//         shooterStep = 0;
//         intakeStep = 0;
//         armStep = 0;
        
        
//     }


//     @Override
//     public void initialize(){

//     }

//     @Override
//     public void execute(){
//         double[] desired = Constants.camera.getDesiredShoot();
//         //shoot a note
//         //drive back
//         //intake
//         //shoot
//         //drive left
//         //intake
//         //shoot
//         //drive left
//         //intake
//         //shoot

//         if(shooterStep == 0){
//             Constants.shooter.setSpeed(desired[1], desired[2]);
//         }

    
       
//     }

//     @Override
//     public void end(boolean interrupted) {

//     }






// }

