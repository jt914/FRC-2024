package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera {
    private PhotonCamera cam;
    public static class LimeLight {
        private NetworkTable table;
        
        public LimeLight() {
            table = NetworkTableInstance.getDefault().getTable("limelight-cam");
        }

        public NetworkTable getTable() {
            return table;
        }

        public double getTid() {
            System.out.println("getTid");
            return table.getEntry("tid").getDouble(0);
        }

        public double[] getTids() {
            return table.getEntry("tid").getDoubleArray(new double[0]);
        }
    }

    

    public Camera(){
        cam = new PhotonCamera("photonvision");
        
    }

    public PhotonPipelineResult getLatestResult(){
        return cam.getLatestResult();
        
    }

    public void end(){
        cam.close();
    }

    //if null the command to get desired shoot should just do nothing
    public double getDesiredShoot(){
        /*
        1. use Camera to get pose from hood
        2. calculate optimal arm angle and shooter speed for camera
        might need to do testing
        
        */

        PhotonTrackedTarget shooterTarget = getLatestResult().getTargets().get(0);
        // PhotonUtils.calculateDistanceToTargetMeters(0, );

        boolean seesShooter = false;

        
        for(PhotonTrackedTarget target: getLatestResult().getTargets()){
            
            if(target.getFiducialId() == 9){
                shooterTarget = target;

                seesShooter = true;
                break;
        }



        // System.out.println(shooterTarget.getYaw());
        // for(PhotonTrackedTarget target: getLatestResult().getTargets()){
        //     if(target.getFiducialId() == 7 || target.getFiducialId() == 4){
        //         shooterTarget = target;
        //         seesShooter = true;
        //         break;
        // }

        // if(!seesShooter){
        //     return null;
        // }
        // return null;




        
        }


        return shooterTarget.getYaw();



        // double[] ret = new double[]{Math.random()* 50, Math.random(), Math.random()};
        // return new double[]{1,0.1,0.1};
        //index 0 is optimal arm angle, index 1 is bot speed, index 2 is top speed, 
        
    }

    public double getDriveOffset(){
        /*
        1. use Camera to get pose from hood
        2. calculate rotation needed to make it
        might need to do testing
        
        */
        return 90;
    }


}
