package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
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

    public double[] getDesiredShoot(){
        /*
        1. use Camera to get pose from hood
        2. calculate optimal arm angle and shooter speed for camera
        might need to do testing
        
        */

        // double[] ret = new double[]{Math.random()* 50, Math.random(), Math.random()};
        return new double[]{1,2,3};
        //index 0 is optimal arm angle, index 1 is bot speed, index 2 is top speed, 
        
    }

    public double getDriveOffset(){
        /*
        1. use Camera to get pose from hood
        2. calculate rotation needed to make it
        might need to do testing
        
        */
        return 0;
    }


}
