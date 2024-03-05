package frc.robot.autoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoNamedCommands {

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;


    public AutoNamedCommands() {
        registerCommands();
    }

    public void registerCommands() {
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> intakeSubsystem.roll(.70)));
    NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> intakeSubsystem.rollStop()));
    NamedCommands.registerCommand("Flywheel", new InstantCommand(() -> shooterSubsystem.shootFlywheel(.3)));
    NamedCommands.registerCommand("Stop Flywheel", new InstantCommand(() -> shooterSubsystem.stopFlywheel()));
    NamedCommands.registerCommand("Bump In", new InstantCommand(() -> shooterSubsystem.spinBump(.2)));
    NamedCommands.registerCommand("Bump Fly", new InstantCommand(() -> shooterSubsystem.spinBump(.4)));
    NamedCommands.registerCommand("Bump Down", new InstantCommand(() -> shooterSubsystem.spinBump(-.2)));
    NamedCommands.registerCommand("Stop Bump", new InstantCommand(() -> shooterSubsystem.stopBump()));


    }

    public static Command getAutoCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);
    }

}