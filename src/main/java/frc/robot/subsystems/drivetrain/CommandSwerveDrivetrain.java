package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    StructArrayPublisher<SwerveModuleState> publisher;

    private final Rotation2d BluePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d RedPerspectiveRotation = Rotation2d.fromDegrees(180);

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private boolean hasAppliedPerspective = false;


    WPI_PigeonIMU gyro = new WPI_PigeonIMU(0);

    private Field2d field = new Field2d();


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("SwerveStates", SwerveModuleState.struct).publish();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }


    public void periodic() {
        publisher.set(super.getState().ModuleStates);

        if(!hasAppliedPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == DriverStation.Alliance.Red ? RedPerspectiveRotation
                                : BluePerspectiveRotation);
                hasAppliedPerspective = true;
            });

    }
}
    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(3, 0, 0.0),
                                            new PIDConstants(3, 0, .0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()->{
                var alliance = DriverStation.getAlliance();
                    if(alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                       } 
          return false;}, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }

                
    
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }


    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void driveLimit() {
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        for (var swerveModule : Modules) {
            TalonFX driveMotor = swerveModule.getDriveMotor();
            driveMotor.getConfigurator().refresh(currentLimits);

            currentLimits.SupplyCurrentLimit = 60;
            currentLimits.SupplyCurrentThreshold = 80;
            currentLimits.SupplyTimeThreshold = 0.5;
            currentLimits.SupplyCurrentLimitEnable = true;
            driveMotor.getConfigurator().apply(currentLimits);
        }
    }

    public void azimuthLimit() {
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        for (var swerveModule : Modules) {
            TalonFX driveMotor = swerveModule.getSteerMotor();
            driveMotor.getConfigurator().refresh(currentLimits);

            currentLimits.SupplyCurrentLimit = 30;
            currentLimits.SupplyCurrentThreshold = 40;
            currentLimits.SupplyTimeThreshold = 0.25;
            currentLimits.SupplyCurrentLimitEnable = true;
            driveMotor.getConfigurator().apply(currentLimits);
        }
    }
    
    public void ZeroGyro(){
        gyro.reset();
    }
}