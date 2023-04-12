package frc.Autonomous;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.Mechanisms.AbstractMechanism;
import frc.Mechanisms.CatzDrivetrain;

public class CatzRobotTracker extends AbstractMechanism{
    private static CatzRobotTracker instance = null;

    private static final int THREAD_PERIOD_MS = 20;

    private SwerveDrivePoseEstimator poseEstimator;

    private final CatzDrivetrain driveTrain = CatzDrivetrain.getInstance();
    private final CatzAprilTag limelight =  CatzAprilTag.getInstance();
    
    private CatzRobotTracker()
    {
        super(THREAD_PERIOD_MS);
    }

    //METHOD USED IN AUTONOMOUS CONTAINER
    public void resetPosition(Pose2d pose)
    {
        driveTrain.resetDriveEncs();
        poseEstimator.resetPosition(Rotation2d.fromDegrees(driveTrain.getGyroAngle()), driveTrain.getModulePositions(), pose);
    }
    //METHOD USED IN AUTONOMOUS CONTAINER

    public static CatzRobotTracker getInstance()
    {
        if(instance == null) 
        {
            instance = new CatzRobotTracker();
        }
        return instance;
    }

    public Pose2d getEstimatedPosition()
    {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void update() 
    {
        if(limelight.aprilTagInView())
        {
            poseEstimator.addVisionMeasurement(limelight.getLimelightBotPose(), Timer.getFPGATimestamp());
        }
        poseEstimator.update(Rotation2d.fromDegrees(driveTrain.getGyroAngle()),driveTrain.getModulePositions()); 
    }

    @Override
    public void smartDashboard() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void smartDashboard_DEBUG() {
        // TODO Auto-generated method stub
        
    }
}
