package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.IntakeConstants;
//import frc.robot.constants.TalonFXConstants;
import frc.robot.constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {
    TalonFX intakeFront;
    TalonFX intakeBack;

    public static IntakeSubsystem m_Instance = null;

    public IntakeSubsystem() {
        intakeFront = new TalonFX(IntakeConstants.INTAKE_FRONT_ID);
        intakeBack = new TalonFX(IntakeConstants.INTAKE_BACK_ID);

        var intakeFrontConfiguration = new TalonFXConfiguration();
        var intakeBackConfiguration = new TalonFXConfiguration();

        intakeFrontConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeFrontConfiguration.CurrentLimits.SupplyCurrentLimit = 55;
        intakeFrontConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeBackConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeBackConfiguration.CurrentLimits.SupplyCurrentLimit = 55;
        intakeBackConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;



        intakeFront.setControl(new Follower(intakeBack.getDeviceID(), true));

    }

    
    public void rollIn(double speed) {
        intakeFront.set(Math.abs(speed));
        intakeBack.set(Math.abs(speed));

        
    }

    public void rollOut(double speed) {
        intakeFront.set(-Math.abs(speed));
        intakeBack.set(-Math.abs(speed));

    }

    public void roll(double speed) {
        intakeFront.set(speed);
        intakeBack.set(speed);
        System.out.println("Intake");

    } 

    public void rollStop() {
        intakeFront.set(0);
        intakeBack.set(0);
        System.out.println("Stop Intake");

    //    System.out.println("STOP");
    }



    
}