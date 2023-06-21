package frc;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.subsystems.SwerveDrive;

public class PoseEstimado implements Runnable {

    private SwerveDrivePoseEstimator poseEstimator = null;
    private SwerveDrive swerveDrive;

    public PoseEstimado() {

    }

    public void configurePoseEstimator(SwerveDrive swerveDrive, SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPose) {
        this.swerveDrive = swerveDrive;
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initialPose);
    }

    @Override
    public void run() {

        boolean initialized = false;

            while(true){
                if (initialized) {

                    System.out.print("HI");
                    //poseEstimator.addVisionMeasurement(null, 0);

                    poseEstimator.update(swerveDrive.getGyroAngle(), swerveDrive.getSwerveModulePositions());
                    
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        System.out.println("Thread failed to sleep");
                    }
                }
                else if (poseEstimator != null) {
                    initialized = true;

                    System.out.print("Initialized");
                }

                
            }
        
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    

}
