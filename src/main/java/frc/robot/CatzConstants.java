package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class CatzConstants {
    private static final double MODULE_DISTANCE_FROM_CENTER = 0.2984;

    private static final Translation2d SWERVE_LEFT_FRONT_LOCATION  = new Translation2d(MODULE_DISTANCE_FROM_CENTER,MODULE_DISTANCE_FROM_CENTER);
    private static final Translation2d SWERVE_LEFT_BACK_LOCATION   = new Translation2d(MODULE_DISTANCE_FROM_CENTER, -MODULE_DISTANCE_FROM_CENTER);
    private static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(-MODULE_DISTANCE_FROM_CENTER, MODULE_DISTANCE_FROM_CENTER);
    private static final Translation2d SWERVE_RIGHT_BACK_LOCATION  = new Translation2d(-MODULE_DISTANCE_FROM_CENTER, -MODULE_DISTANCE_FROM_CENTER);

    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        SWERVE_LEFT_FRONT_LOCATION,
        SWERVE_LEFT_BACK_LOCATION,
        SWERVE_RIGHT_FRONT_LOCATION,
        SWERVE_RIGHT_BACK_LOCATION
    );

    public static final double MAX_SPEED = 4.0;

    public static final double SDS_L1_GEAR_RATIO = 8.14;       //SDS mk4i L1 ratio
    public static final double SDS_L2_GEAR_RATIO = 6.75;       //SDS mk4i L2 ratio
    
    public static final double DRVTRAIN_WHEEL_DIAMETER             = 4.0;
    public static final double DRVTRAIN_WHEEL_CIRCUMFERENCE        = (Math.PI * DRVTRAIN_WHEEL_DIAMETER);
}
