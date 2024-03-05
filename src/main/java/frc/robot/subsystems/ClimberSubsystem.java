/* package frc.robot.subsystems;


//import com.ctre.phoenix6.StatusSignal;

//import com.ctre.phoenix6.configs.CANcoderConfiguration;
//import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.Follower;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climberFalcon1;

    public ClimberSubsystem() {
        climberFalcon1 = new TalonFX(ClimberConstants.CLIMBER_TALON_1);
    }

    @Override
    public void periodic() {
    }

    public void climbUp(double speed) {
        climberFalcon1.set(speed);
    }

    public void climbDown(double speed) {
        climberFalcon1.set(-speed);
    }

    public void stopClimb() {
        climberFalcon1.set(0);
    
    }

    public double getRightEncoder() {
        return climberFalcon1.getRotorPosition().getValue();

    }


}

*/