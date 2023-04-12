package frc.Autonomous;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.Mechanisms.AbstractMechanism;
import frc.Mechanisms.CatzDrivetrain;
import frc.robot.CatzConstants;
import frc.robot.Robot;

public class CatzAutonomous extends AbstractMechanism{
    private static CatzAutonomous instance = null;

    private static final int THREAD_PERIOD_MS = 20;

    private final SwerveDriveKinematics kinematics = CatzConstants.swerveDriveKinematics;

    private final HolonomicDriveController holonomicDriveController;

    private boolean wantsVisionAlign = false;

    private double autoStartTime;
    private volatile boolean isFinished = false;

    private Trajectory currentTrajectory;
    private Rotation2d targetRotation;

    private final ChassisSpeeds nullChassisSpeeds = new ChassisSpeeds(0,0,0);
    private final SwerveModuleState[] nullModuleStates = kinematics.toSwerveModuleStates(nullChassisSpeeds);

    private final CatzDrivetrain driveTrain = CatzDrivetrain.getInstance();
    private final CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
    private final CatzAprilTag limelight = CatzAprilTag.getInstance();

    private CatzAutonomous()
    {
        super(THREAD_PERIOD_MS);

        final PIDController xController = new PIDController(0.2, 0, 0);
        final PIDController yController = new PIDController(0.2, 0, 0);

        final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(2.5,Math.pow(2.5,2));
        final ProfiledPIDController thetaController = new ProfiledPIDController(5.0, 0, 0, thetaControllerConstraints);

        holonomicDriveController = new HolonomicDriveController(
            xController,
            yController,
            thetaController
        );
    }

    //METHODS USED IN AUTONOMOUS CONTAINER
    public void setAutoPath(Trajectory newTrajectory)
    {
        isFinished = false;
        currentTrajectory = newTrajectory;
        autoStartTime = Robot.getAutonTimer().get();
    }

    public void setAutoRotation(Rotation2d newRotation)
    {
        isFinished = false;
        targetRotation = newRotation;
    }

    public void stopMovement()
    {
        isFinished = true;
    }


    public boolean isFinished()
    {
        isFinished = Robot.getAutonTimer().hasElapsed(currentTrajectory.getTotalTimeSeconds());

        return isFinished;
    }

    public double getAutoElapsedTime()
    {
        return Robot.getAutonTimer().get() - autoStartTime;
    }
    //METHODS USED IN AUTONOMOUS CONTAINER

    public static CatzAutonomous getInstance()
    {
        if(instance == null)
        {
            instance = new CatzAutonomous();
        }
        return instance;
    }

    //to be called in the GUI
    public void setWantVisionAlign(boolean state)
    {
        wantsVisionAlign = state;
    }

    public boolean getWantsVisionAlign()
    {
        return wantsVisionAlign;
    }

    private void runAuto()
    {
        try
        {
            double currentTime = Robot.getAutonTimer().get();
            Trajectory.State goal = currentTrajectory.sample(currentTime - autoStartTime);
            Rotation2d desiredRotation = new Rotation2d();

            if(wantsVisionAlign)
            {
                //do vision align
            }
            else
            {
                desiredRotation = targetRotation;
            }

            ChassisSpeeds adjustedSpeed = holonomicDriveController.calculate(robotTracker.getEstimatedPosition(), goal, desiredRotation);
            SwerveModuleState[] targetModuleStates = kinematics.toSwerveModuleStates(adjustedSpeed);

            driveTrain.setSwerveModuleStates(targetModuleStates);
        }
        catch(NullPointerException e){}
    }

    @Override
    public void update() {
        if(!isFinished)
        {
            runAuto();
        }
        else
        {
            driveTrain.setSwerveModuleStates(nullModuleStates);
        }
        
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
