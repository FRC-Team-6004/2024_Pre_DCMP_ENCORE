package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TalonFXConstants;

//import static frc.robot.constants.ShooterConstants.MOTION_MAGIC_ACCELERATION;
//import static frc.robot.constants.ShooterConstants.SHOOTER_F;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX topFalcon;
    private final TalonFX bottomFalcon;
    private final TalonFX bumpFalcon;
    private final DigitalInput beamBreak;
    public enum ShooterModes {
        DEFAULT,
        SPEAKER,
        AMP,
        TRAP,
        BUMP
    }

    public static ShooterModes shooterModes;

    public enum ShooterStatus {
        FORWARD,
        BACKWARD,
        OFF
    }

    public static ShooterStatus shooterStatus;
    public static double setpoint = 0;
    public static boolean atSetpoint = false;
    public static boolean isShooting = false;
    public static double velocity = 0;
    MotionMagicVelocityVoltage motionMagicVelocityVoltage;



    public ShooterSubsystem() {
        topFalcon = new TalonFX(ShooterConstants.SHOOTER_TOP_MOTOR);
        bottomFalcon = new TalonFX(ShooterConstants.SHOOTER_BOTTOM_MOTOR);
        var bottomMotorConfigurator = bottomFalcon.getConfigurator();
        var topMotorConfigurator = topFalcon.getConfigurator();
        var bottomMotorConfiguration = new TalonFXConfiguration();
        
        bottomMotorConfiguration.MotorOutput.Inverted = ShooterConstants.SHOOTER_INVERSION;
        bottomMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        bottomMotorConfiguration.CurrentLimits = ShooterConstants.SHOOTER_CURRENT_LIMIT;
        bottomMotorConfiguration.Slot0 = ShooterConstants.SLOT_0_CONFIGS;
        topFalcon.setControl(new Follower(bottomFalcon.getDeviceID(), true));
        shooterStatus = ShooterStatus.OFF;
         shooterModes = ShooterModes.TRAP;




         bumpFalcon = new TalonFX(ShooterConstants.SHOOTER_BUMP);

         beamBreak = new DigitalInput(ShooterConstants.BEAM_BREAK);


    }

    @Override
 public void periodic() {

   }
    public boolean getBeamBreak(){
        return beamBreak.get();
    }

    public void spinBump(double speed){
        bumpFalcon.set(speed);
        System.out.println("Spin Bump");

    }

    public void shootFlywheelVolts(double voltage){
        bottomFalcon.setVoltage(-setpoint);
        topFalcon.setVoltage(setpoint);

    }

    public void getBottomVelocity(){
        bottomFalcon.getVelocity();
    }

    public void getTopVelocity(){
        topFalcon.getVelocity();
    }


    public void shootFlywheel(double speed) {
        bottomFalcon.set(-speed);
        topFalcon.set(speed);
        System.out.println("Shoot");

      //  bumpFalcon.set(0.9); //
    // shooterStatus = ShooterStatus.FORWARD;
    }

    public void setMode(ShooterModes mode) {
        shooterModes = mode;
    }


    public void stopBump(){
        bumpFalcon.stopMotor();
        System.out.println("Stop Bump");
    }

    public void stopFlywheel() {
        bottomFalcon.set(0);
        topFalcon.set(0);
       // bumpFalcon.stopMotor();
        System.out.println("Stop Shoot");

        shooterStatus = ShooterStatus.OFF;
    }

    public double getBottomEncoder() {
        return bottomFalcon.getRotorVelocity().getValue();
    }

    public  double getTopEncoder() {
        return topFalcon.getRotorVelocity().getValue();
    }

    public double getAverageEncoder() {
        return (getTopEncoder() + getBottomEncoder()) / 2;
    }

    public double getFlywheelRPM() {
        return getAverageEncoder() * ShooterConstants.PULLEY_RATIO * (600.0 / TalonFXConstants.COUNTS_PER_REV);
    }

    public double flywheelMPS(double rpm) {
        return (ShooterConstants.FLYWHEEL_DIAMETER_M * Math.PI) * (rpm * 60.0);
    }

    public void setSetpoint(double setpoint) {
        ShooterSubsystem.setpoint = setpoint;
    }

    public void setVelocity(double vel){

        topFalcon.setControl(motionMagicVelocityVoltage.withVelocity(vel));
        bottomFalcon.setControl(motionMagicVelocityVoltage.withVelocity(vel));
     //   System.out.println((getFlywheelRPM()));

}
}