// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.lang.model.element.Parameterizable;

//import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import frc.robot.util.LimelightHelpers;
//import frc.robot.util.controllerUtils.MultiButton;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.*;



public class RobotContainer {
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();

    

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // 6 meters per second desired top speed change the decimal for speeding up
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driveStick = new CommandXboxController(0); //drivestick
  private final CommandXboxController opStick = new CommandXboxController(1); // My joystick

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  SendableChooser<Command> autoChooserAuto = new SendableChooser<>();
  SendableChooser<Command> choiceWait = new SendableChooser<>();
  
  private final Command Wait0 = new WaitCommand(0);
  private final Command Wait1 = new WaitCommand(1);
  private final Command Wait2 = new WaitCommand(2);
  private final Command Wait3 = new WaitCommand(3);
  private final Command Wait4 = new WaitCommand(4);
  private final Command Wait5 = new WaitCommand(5);
  private final Command Wait6 = new WaitCommand(6);
  private final Command Wait7 = new WaitCommand(7);
  private final Command Wait8 = new WaitCommand(8);
  private final Command Wait9 = new WaitCommand(9);
  private final Command Wait10 = new WaitCommand(10);

  private final Command Top2Piece = new SequentialCommandGroup(
          new ParallelCommandGroup(
              new PathPlannerAuto("Top Start"),
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.29))),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> intakeSubsystem.roll(1)),
           new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
           new PathPlannerAuto("Top Close Note"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.15),
          // new InstantCommand(()-> intakeSubsystem.rollStop()),
           new ParallelCommandGroup(
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
              new PathPlannerAuto("Top Close to Shoot")),
          //new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
          new InstantCommand(()-> shooterSubsystem.stopBump()),
          new InstantCommand(()-> intakeSubsystem.rollStop())
          );
  
  private final Command Top3Close = new SequentialCommandGroup(
     new ParallelCommandGroup(
              new PathPlannerAuto("Top Start"),
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.29))),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> intakeSubsystem.roll(1)),
           new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
           new PathPlannerAuto("Top Close Note"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.15),
          // new InstantCommand(()-> intakeSubsystem.rollStop()),
           new ParallelCommandGroup(
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
              new PathPlannerAuto("Top Close to Shoot")),
          //new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(.5),
          new InstantCommand(()-> intakeSubsystem.roll(1)),
         new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
         new PathPlannerAuto("Shoot To Mid Close"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.15),
           new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
          new PathPlannerAuto("Mid Close to Shoot"),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
          new InstantCommand(()-> shooterSubsystem.stopBump()),
          new InstantCommand(()-> intakeSubsystem.rollStop())
  );

  private final Command Top3TopMid = new SequentialCommandGroup(
    new ParallelCommandGroup(
              new PathPlannerAuto("Top Start"),
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.29))),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> intakeSubsystem.roll(1)),
           new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
           new PathPlannerAuto("Top Close Note"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.15),
          // new InstantCommand(()-> intakeSubsystem.rollStop()),
           new ParallelCommandGroup(
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
              new PathPlannerAuto("Top Close to Shoot")),
          //new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(.5),
          new InstantCommand(()-> intakeSubsystem.roll(1)),
         new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
         new PathPlannerAuto("Top Shoot to Mid Top"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.15),
           new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
          new PathPlannerAuto("Mid Top to Shoot"),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
          new InstantCommand(()-> shooterSubsystem.stopBump()),
          new InstantCommand(()-> intakeSubsystem.rollStop())
  );



  private final Command Top4Piece2CloseTopMid = new SequentialCommandGroup(
      new ParallelCommandGroup(
              new PathPlannerAuto("Top Start"),
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.29))),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> intakeSubsystem.roll(1)),
           new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
           new PathPlannerAuto("Top Close Note"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.15),
          // new InstantCommand(()-> intakeSubsystem.rollStop()),
           new ParallelCommandGroup(
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
              new PathPlannerAuto("Top Close to Shoot")),
          //new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(.5),
          new InstantCommand(()-> intakeSubsystem.roll(1)),
         new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
          new PathPlannerAuto("Shoot To Mid Close"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.15),
           new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
          new PathPlannerAuto("Mid Close to Shoot"),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(.5),
         new PathPlannerAuto("Top Shoot to Mid Top"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.15),
           new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
          new PathPlannerAuto("Mid Top to Shoot"),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
          new InstantCommand(()-> shooterSubsystem.stopBump()),
          new InstantCommand(()-> intakeSubsystem.rollStop())
  );


  private final Command Top4PieceCloseAll = new SequentialCommandGroup(
new ParallelCommandGroup(
              new PathPlannerAuto("Top Start"),
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.29))),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> intakeSubsystem.roll(1)),
           new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
           new PathPlannerAuto("Top Close Note"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.15),
          // new InstantCommand(()-> intakeSubsystem.rollStop()),
           new ParallelCommandGroup(
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
              new PathPlannerAuto("Top Close to Shoot")),
          new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(.5),
          new InstantCommand(()-> intakeSubsystem.roll(1)),
         new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
            new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
          new PathPlannerAuto("Shoot To Mid Close"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.15),
           new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
          new PathPlannerAuto("Mid Close to Shoot"),
          new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(.5),
         new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
         new PathPlannerAuto("Mid Shoot to Close Bottom"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.15),
           new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
          new PathPlannerAuto("Close Bottom to Shoot"),
          new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
          new InstantCommand(()-> shooterSubsystem.stopBump()),
          new InstantCommand(()-> intakeSubsystem.rollStop())
  );

  private final Command Top5Piece2Close2TopMid =new SequentialCommandGroup(
     new PathPlannerAuto("Top Start"),
     new PathPlannerAuto("Top Close Note"),
     new PathPlannerAuto("Top Close to Shoot"),
     new PathPlannerAuto("Shoot To Mid Close"),
     new PathPlannerAuto("Mid Close to Shoot"),
     new PathPlannerAuto("Top Shoot to Mid Top"),
     new PathPlannerAuto("Mid Top to Shoot"),
     new PathPlannerAuto("Top Shoot to Mid Top Down"),
     new PathPlannerAuto("Mid Top Down to Shoot")
  );



  private final Command Bottom2Piece = new SequentialCommandGroup(
     new PathPlannerAuto("Bottom Start"),
     new PathPlannerAuto("Bottom Close"),
     new PathPlannerAuto("Bottom Shoot")
  );

  private final Command Bottom3Close = new SequentialCommandGroup(
     new PathPlannerAuto("Bottom Start"),
     new PathPlannerAuto("Bottom Close"),
     new PathPlannerAuto("Bottom Shoot"),
     new PathPlannerAuto("Bottom Shoot To Mid Close"),
     new PathPlannerAuto("Mid Close to Shoot")
  );

  private final Command Bottom3Mid = new SequentialCommandGroup(
     new PathPlannerAuto("Bottom Start"),
     new PathPlannerAuto("Bottom Close"),
     new PathPlannerAuto("Bottom Shoot"),
     new PathPlannerAuto("BottomMidBottom"),
     new PathPlannerAuto("BottomMidBottom to shoot")
  );
  private final Command Bottom4Close2Mid = new SequentialCommandGroup(
     new PathPlannerAuto("Bottom Start"),
     new PathPlannerAuto("Bottom Close"),
     new PathPlannerAuto("Bottom Shoot"),
     new PathPlannerAuto("Bottom Shoot To Mid Close"),
     new PathPlannerAuto("Mid Close to Shoot"),
     new PathPlannerAuto("BottomMidBottom"),
     new PathPlannerAuto("BottomMidBottom to shoot")
  );
  private final Command Bottom4CloseAll = new SequentialCommandGroup(
     new PathPlannerAuto("Bottom Start"),
     new PathPlannerAuto("Bottom Close"),
     new PathPlannerAuto("Bottom Shoot"),
     new PathPlannerAuto("Bottom Shoot To Mid Close"),
     new PathPlannerAuto("Mid Close to Shoot"),
     new PathPlannerAuto("Top Close Note"),
     new PathPlannerAuto("Top Close to Shoot")
  );


  private final Command Mid2 = new SequentialCommandGroup(
     new PathPlannerAuto("Mid Start"),
     new PathPlannerAuto("Mid Pick"),
     new PathPlannerAuto("MidShoot")
  );

  private final Command Mid3CenterMid = new SequentialCommandGroup(
     new PathPlannerAuto("Mid Start"),
     new PathPlannerAuto("Mid Pick"),
     new PathPlannerAuto("MidShoot"),
     new PathPlannerAuto("Mid Middle Pick"),
     new PathPlannerAuto("Mid Middle Shoot")
  );

  private final Command Mid4CenterMidBelow = new SequentialCommandGroup(
     new PathPlannerAuto("Mid Start"),
     new PathPlannerAuto("Mid Pick"),
     new PathPlannerAuto("MidShoot"),
     new PathPlannerAuto("Mid Middle Pick"),
     new PathPlannerAuto("Mid Middle Shoot"),
     new PathPlannerAuto("Mid Middle Down Pick"),
     new PathPlannerAuto("Mid Middle Down Shoot")
  );

  private final Command Mid4CenterMidUp = new SequentialCommandGroup(
     new PathPlannerAuto("Mid Start"),
     new PathPlannerAuto("Mid Pick"),
     new PathPlannerAuto("MidShoot"),
     new PathPlannerAuto("Mid Middle Pick"),
     new PathPlannerAuto("Mid Middle Shoot"),
     new PathPlannerAuto("Mid Middle Up Pick"),
     new PathPlannerAuto("Mid Middle Up Shoot")
  );

  private final Command Mid5CenterMid3 = new SequentialCommandGroup(
     new PathPlannerAuto("Mid Start"),
     new PathPlannerAuto("Mid Pick"),
     new PathPlannerAuto("MidShoot"),
     new PathPlannerAuto("Mid Middle Pick"),
     new PathPlannerAuto("Mid Middle Shoot"),
     new PathPlannerAuto("Mid Middle Up Pick"),
     new PathPlannerAuto("Mid Middle Up Shoot"),
     new PathPlannerAuto("Mid Middle Down Pick"),
     new PathPlannerAuto("Mid Middle Down Shoot")
  );

private final Command BottomMidRush = new SequentialCommandGroup(
     new PathPlannerAuto("MidRush1"),
     new PathPlannerAuto("MidRush2"),
     new PathPlannerAuto("MidRush3"),
     new PathPlannerAuto("MidRush4"),
     new PathPlannerAuto("MidRush5"),
     new PathPlannerAuto("MidRush6"),
     new PathPlannerAuto("MidRush7")
  );


  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .0175;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight-shooter") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= MaxAngularRate;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .25;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight-shooter") * kP;
    targetingForwardSpeed *= MaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

// simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional_intake()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .0175;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocityIntake = LimelightHelpers.getTX("limelight-intake") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocityIntake *= MaxAngularRate;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocityIntake *= -1.0;

    return targetingAngularVelocityIntake;
  }

   double limelight_range_proportional_intake()
  {    
    double kP = -.25;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight-intake") * kP;
    targetingForwardSpeed *= MaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  private void configureBindings() {

    //Driver Xbox Controller
  // default command
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveStick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driveStick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
//slow command
    driveStick.leftTrigger().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveStick.getLeftY() * MaxSpeed*(.5)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed*(.5)) // Drive left with negative X (left)
            .withRotationalRate(-driveStick.getRightX() * MaxAngularRate*(.35) // Drive counterclockwise with negative X (left)
        )));    
    
  //slow command while intaking  
    driveStick.rightTrigger().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveStick.getLeftY() * MaxSpeed*(.6)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed*(.35)) // Drive left with negative X (left)
            .withRotationalRate(-driveStick.getRightX() * MaxAngularRate*(.6) // Drive counterclockwise with negative X (left)
        )));       
   
   driveStick.rightTrigger().whileTrue(
      new StartEndCommand(()-> intakeSubsystem.roll(.3), intakeSubsystem::rollStop));

   driveStick.rightTrigger().whileTrue(
      new StartEndCommand(() -> shooterSubsystem.spinBump(.5), shooterSubsystem::stopBump));     

   driveStick.rightTrigger().whileTrue(
      new StartEndCommand(() -> shooterSubsystem.shootFlywheel(-.075), shooterSubsystem::stopBump));  

   driveStick.rightBumper().whileTrue(
      new StartEndCommand(() -> shooterSubsystem.shootFlywheel(-.075), shooterSubsystem::stopBump));  
// Auto pickup
    driveStick.rightBumper().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX((-driveStick.getLeftY() * MaxSpeed*(.6))) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed*(.50)) // Drive left with negative X (left)
            .withRotationalRate(limelight_aim_proportional_intake() // Drive counterclockwise with negative X (left)
        )));  
   driveStick.rightBumper().whileTrue(
      new StartEndCommand(()-> intakeSubsystem.roll(1), intakeSubsystem::rollStop));

   driveStick.rightBumper().whileTrue(
      new StartEndCommand(() -> shooterSubsystem.spinBump(.7), shooterSubsystem::stopBump));       


    driveStick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driveStick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driveStick.getLeftY(), -driveStick.getLeftX()))));

    driveStick.leftBumper().whileTrue(

    //lime light aim speaker
    drivetrain.applyRequest(() -> drive.withVelocityX(limelight_range_proportional()*MaxSpeed)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed*(.4))
            .withRotationalRate(limelight_aim_proportional())));
            
    //reset the field-centric heading on y button press
    driveStick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    //Op Xbox Controller
    
    opStick.x().whileTrue(
    new StartEndCommand(() -> shooterSubsystem.shootFlywheel(.335), shooterSubsystem::stopFlywheel));

    opStick.leftTrigger().whileTrue(
    new StartEndCommand(() -> shooterSubsystem.shootFlywheel(.14), shooterSubsystem::stopFlywheel));

    opStick.b().whileTrue(
       new StartEndCommand(() -> shooterSubsystem.spinBump(-.2), shooterSubsystem::stopBump));
      
    opStick.a().whileTrue(
       new StartEndCommand(() -> shooterSubsystem.spinBump(.2), shooterSubsystem::stopBump));

    opStick.y().whileTrue(
       new StartEndCommand(() -> shooterSubsystem.spinBump(-.5), shooterSubsystem::stopBump));

    opStick.y().whileTrue(
       new StartEndCommand(() -> intakeSubsystem.roll(-.5), intakeSubsystem::rollStop));

    
     
  //  opStick.rightTrigger().whileTrue( 
  //    new StartEndCommand(() -> climberSubsystem.climbUp(.2), climberSubsystem::stopClimb));
  //   opStick.leftTrigger().whileTrue( 
  //    new StartEndCommand(() -> climberSubsystem.climbDown(.2), climberSubsystem::stopClimb));

  }


  public RobotContainer() {
    configureBindings();
   
    autoChooserAuto.setDefaultOption("Top4Piece2CloseTopMid", Top4Piece2CloseTopMid);
    autoChooserAuto.addOption("Top2Piece", Top2Piece);
    autoChooserAuto.addOption("Top3Close", Top3Close);
    autoChooserAuto.addOption("Top3TopMid", Top3TopMid);
    autoChooserAuto.addOption("Top4PieceCloseAll", Top4PieceCloseAll);
    autoChooserAuto.addOption("Top5Piece2Close2TopMid", Top5Piece2Close2TopMid);

    autoChooserAuto.addOption("Bottom2Piece", Bottom2Piece);
    autoChooserAuto.addOption("Bottom3Close", Bottom3Close);
    autoChooserAuto.addOption("Bottom3Mid", Bottom3Mid);
    autoChooserAuto.addOption("Bottom4Close2Mid", Bottom4Close2Mid);
    autoChooserAuto.addOption("Bottom4CloseAll", Bottom4CloseAll);
    autoChooserAuto.addOption("BottomMidRush", BottomMidRush);

    autoChooserAuto.addOption("Mid2", Mid2);
    autoChooserAuto.addOption("Mid3CenterMid", Mid3CenterMid);
    autoChooserAuto.addOption("Mid4CenterMidBelow", Mid4CenterMidBelow);
    autoChooserAuto.addOption("Mid4CenterMidUp", Mid4CenterMidUp);
    autoChooserAuto.addOption("Mid5CenterMid3", Mid5CenterMid3);

   choiceWait.setDefaultOption("0 Seconds", Wait0);
   choiceWait.addOption("1 Seconds", Wait1);
   choiceWait.addOption("2 Seconds", Wait2);
   choiceWait.addOption("3 Seconds", Wait3);
   choiceWait.addOption("4 Seconds", Wait4);
   choiceWait.addOption("5 Seconds", Wait5);
   choiceWait.addOption("6 Seconds", Wait6);
   choiceWait.addOption("7 Seconds", Wait7);
   choiceWait.addOption("8 Seconds", Wait8);
   choiceWait.addOption("9 Seconds", Wait9);
   choiceWait.addOption("10 Seconds", Wait10);


   
   SmartDashboard.putData(choiceWait);
   SmartDashboard.putData(autoChooserAuto);


  // SmartDashboard.putData(autoChooserAuto);

  }

    public Command getAutonomousCommand() {
      
return new PathPlannerAuto("Forward");
   /*    return new SequentialCommandGroup(
      choiceWait.getSelected(),
      autoChooserAuto.getSelected()
      );
 */
      

/* 

      return new SequentialCommandGroup(
           new ParallelCommandGroup(
              new PathPlannerAuto("Blue Shoot First"),
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.29))),
           new WaitCommand(1.5),
           new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> intakeSubsystem.roll(1)),
           new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
           new PathPlannerAuto("Speaker to Top Close Blue"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.05),
           new ParallelCommandGroup(
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
              new PathPlannerAuto("Shoot 2 Blue")),
          new WaitCommand(1.2),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(1),
          new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
          new InstantCommand(()-> shooterSubsystem.stopBump()),
          new InstantCommand(()-> intakeSubsystem.rollStop())
          );

      */

     // return new PathPlannerAuto("Top Blue");
}
}