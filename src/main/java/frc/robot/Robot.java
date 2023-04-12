// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import com.dacubeking.AutoBuilder.robot.robotinterface.CommandTranslator;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Autonomous.CatzAutonomous;
import frc.Autonomous.CatzRobotTracker;
import frc.Mechanisms.CatzDrivetrain;


public class Robot extends TimedRobot {
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final SendableChooser<String> sideChooser = new SendableChooser<>();

  private final CatzAutonomous auton = CatzAutonomous.getInstance();
  private final CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  private final CatzDrivetrain drivetrain = CatzDrivetrain.getInstance();

  private static final Timer autonTimer = new Timer();
  @Override
  public void robotInit()
  {
    AutonomousContainer.getInstance().initialize(
                true,
                new CommandTranslator(
                        auton::setAutoPath,
                        auton::stopMovement,
                        auton::setAutoRotation,
                        auton::isFinished,
                        auton::getAutoElapsedTime,
                        robotTracker::resetPosition,
                        true
                ),
                false,
                this
        );

      AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));

      sideChooser.setDefaultOption("Blue", "blue");
      sideChooser.addOption("Red", "red");

      SmartDashboard.putData("Path Choices", autoChooser);
      SmartDashboard.putData("Red or Blue", sideChooser);
  }

  @Override
  public void autonomousInit()
  {
    autonTimer.reset();
    autonTimer.start();

    AutonomousContainer.getInstance().runAutonomous(autoChooser.getSelected(), sideChooser.getSelected(), true);
  }

  @Override
  public void disabledInit()
  {
    autonTimer.stop();
  }

  @Override
  public  void testInit()
  {
    drivetrain.initializeOffsets();
  }

  public static Timer getAutonTimer()
  {
    return autonTimer;
  }
}
