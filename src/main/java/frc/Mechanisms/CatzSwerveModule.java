package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.Utils.CatzMathUtils;
import frc.robot.CatzConstants;

public class CatzSwerveModule {
    private final CANSparkMax STEER_MOTOR;
    private final WPI_TalonFX DRIVE_MOTOR;

    private final PIDController steeringPID;
    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private final int motorID;

    private DutyCycleEncoder magEnc;
    private DigitalInput MagEncPWMInput;

    private double wheelOffset;

    private SupplyCurrentLimitConfiguration swerveModuleCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final int     STEER_CURRENT_LIMIT_AMPS      = 30;

    public CatzSwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double wheelOffset)
    {
        STEER_MOTOR = new CANSparkMax(steerMotorID, MotorType.kBrushless);
        DRIVE_MOTOR = new WPI_TalonFX(driveMotorID);

        swerveModuleCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        STEER_MOTOR.setSmartCurrentLimit(STEER_CURRENT_LIMIT_AMPS);
        DRIVE_MOTOR.configSupplyCurrentLimit(swerveModuleCurrentLimit);

        STEER_MOTOR.setIdleMode(IdleMode.kCoast);
        DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);

        steeringPID = new PIDController(kP, kI, kD);
        
        MagEncPWMInput = new DigitalInput(encoderDIOChannel);
        magEnc = new DutyCycleEncoder(MagEncPWMInput);

        this.wheelOffset = wheelOffset;
        this.motorID = steerMotorID; //for smartdashboard
    }

    public void setDesiredState(SwerveModuleState desiredState)
    {
        desiredState = SwerveModuleState.optimize(desiredState, getCurrentRotation()); //optimizes wheel rotation so that the furthest a wheel will ever rotate is 90 degrees.

        double percentOutput = desiredState.speedMetersPerSecond / CatzConstants.MAX_SPEED;
        
        DRIVE_MOTOR.set(ControlMode.PercentOutput, percentOutput);

        double targetAngle = (Math.abs(desiredState.speedMetersPerSecond) <= (CatzConstants.MAX_SPEED * 0.01)) ? getCurrentRotation().getDegrees() : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        double angleError = CatzMathUtils.closestAngle(getCurrentRotation().getDegrees(), targetAngle);

        double command = steeringPID.calculate(getCurrentRotation().getDegrees(), angleError);
        command = -command / (180 * kP); //scale down command to a range of -1 to 1
        STEER_MOTOR.set(command);
    }

    public void resetMagEnc()
    {
        magEnc.reset();
    }

    public void resetDriveEncs()
    {
        DRIVE_MOTOR.setSelectedSensorPosition(0);
    }

    public void initializeOffset()
    {
        wheelOffset = magEnc.get();
    }

    private Rotation2d getCurrentRotation()
    {
        return Rotation2d.fromDegrees(magEnc.get() - wheelOffset);
    }

    public SwerveModuleState getModuleState()
    {
        double velocity = CatzMathUtils.velocityCntsToMPS(DRIVE_MOTOR.getSelectedSensorVelocity(),CatzConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, CatzConstants.SDS_L1_GEAR_RATIO);
        
        return new SwerveModuleState(velocity, getCurrentRotation());
    }

    public SwerveModulePosition getModulePosition()
    {
        return new SwerveModulePosition(Units.inchesToMeters(getDriveDistanceInch()), Rotation2d.fromDegrees(getCurrentRotation().getDegrees()));
    }

    public double getDriveDistanceInch()
    {
        return DRIVE_MOTOR.getSelectedSensorPosition() * CatzConstants.SDS_L1_GEAR_RATIO * CatzConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE / 2048.0;
    }
}
