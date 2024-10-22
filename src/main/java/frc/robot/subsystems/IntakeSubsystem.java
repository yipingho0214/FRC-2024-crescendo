package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.modules.*;
import frc.robot.Constants.PortID;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
// neo*2
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

  public TalonFxMotorPIDmodule up = new TalonFxMotorPIDmodule(PortID.intake_suck_up_kraken,
      NeutralModeValue.Coast, 10, -10);
  public TalonFxMotorPIDmodule down = new TalonFxMotorPIDmodule(PortID.intake_suck_down_kraken,
      NeutralModeValue.Coast, 10, -10);
  
  
  public DigitalInput digitalInput = new DigitalInput(PortID.digital_intake.port);

  SlewRateLimiter filter = new SlewRateLimiter(0.85);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake get?", press());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void stop() {
    up.setPercentOutput(0);
    down.setPercentOutput(0);
  }

  public void take() {
    if (press()) {
      stop();
      return;
    }
    up.setPercentOutput(0.2);
    down.setPercentOutput(0.3);
  }

  public void shoot() {
    up.setPercentOutput(0.5);
    down.setPercentOutput(0.5);
  }

  public void eject() {
    up.setPercentOutput(-0.4);
    down.setPercentOutput(-0.5);
  }

  public boolean press() {
    return !digitalInput.get();
  }

  public void set_NeutralMode(NeutralModeValue mode){
    up.motor.setNeutralMode(mode);
    down.motor.setNeutralMode(mode);
  }
}