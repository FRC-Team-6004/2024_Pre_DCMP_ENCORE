package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.IntakeConstants;
//import frc.robot.constants.TalonFXConstants;

public class IntakeSubsystem extends SubsystemBase {
    TalonFX intakeFront;
    TalonFX intakeBack;

    public static IntakeSubsystem m_Instance = null;

    public IntakeSubsystem() {
        intakeFront = new TalonFX(IntakeConstants.INTAKE_FRONT_ID);
        intakeBack = new TalonFX(IntakeConstants.INTAKE_BACK_ID);


 /*     var intakeConfigurator = intakeFront.getConfigurator();
        var configs = new TalonFXConfiguration();

        configs.MotorOutput.Inverted = IntakeConstants.INTAKE_INVERSION;
        configs.MotorOutput.NeutralMode = IntakeConstants.INTAKE_NEUTRAL_MODE;
        intakeFront.getRotorVelocity().waitForUpdate(IntakeConstants.INTAKE_VELOCITY_STATUS_FRAME);
        intakeFront.getRotorPosition().waitForUpdate(IntakeConstants.INTAKE_POSITION_STATUS_FRAME);
        intakeConfigurator.apply(configs);
  */      
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