// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LimelightHelpers;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  LimelightHelpers.Results lastResult;
  boolean buildAuto = true;


  private final boolean UseLimelight = true;

  @Override

  
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();  
    lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults; 

  }

  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }

  @Override
  public void autonomousPeriodic() {
/* 
       if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == (DriverStation.Alliance.Red)) {
                if (lastResult != null && lastResult.valid) {
                    Pose2d llPose = lastResult.getBotPose2d_wpiRed();
                    m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
                }
            } else {
                if (lastResult != null && lastResult.valid) {
                    Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
                    m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
                }
            }
        }  */
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == (DriverStation.Alliance.Red)) {
          if (lastResult != null && lastResult.valid) {
              Pose2d llPose = lastResult.getBotPose2d_wpiRed();
              m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
          }
      } else {
          if (lastResult != null && lastResult.valid) {
              Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
              m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
          }
      }
  }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
