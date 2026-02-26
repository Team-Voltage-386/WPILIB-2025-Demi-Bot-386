package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.flywheel.FlywheelConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import org.littletonrobotics.junction.Logger;

/** Turret IO on a CTRE Talon SRX Controller. */
public class FlywheelIOTalonSRX implements FlywheelIO {
  // real robot will likely have two motors, use follower
  private final TalonSRX flywheelMotor = new TalonSRX(flywheelCanId);
  private final TalonSRX feederMotor = new TalonSRX(feederCanId);
  private TalonSRXConfiguration flywheelConfig;
  private TalonSRXConfiguration feederConfig;

  private double outputVoltage = 0.0;

  public FlywheelIOTalonSRX() {
    flywheelConfig = new TalonSRXConfiguration();
    flywheelConfig.peakCurrentLimit = currentLimit;
    flywheelConfig.continuousCurrentLimit = currentLimit - 15;
    flywheelConfig.voltageCompSaturation = 12.0;
    tryUntilOkV5(5, () -> flywheelMotor.configAllSettings(flywheelConfig));
    flywheelMotor.setInverted(flywheelInverted);

    feederConfig = new TalonSRXConfiguration();
    // feederConfig.peakCurrentLimit = currentLimit;
    // feederConfig.continuousCurrentLimit = currentLimit - 15;
    // feederConfig.voltageCompSaturation = 12.0;
    tryUntilOkV5(5, () -> feederMotor.configAllSettings(feederConfig));
    feederMotor.setInverted(feederInverted);
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.connected = true;
    inputs.flywheelSpeed = RPM.of(0);
    inputs.outputVoltage = outputVoltage;
    Logger.recordOutput("/Shooter/Flywheel/VelocitySetpoint", 0.0);
    Logger.recordOutput("/Shooter/Flywheel/Velocity", 0.0);
    Logger.recordOutput("/Shooter/Flywheel/VoltageSetpoint", inputs.outputVoltage);
    Logger.recordOutput(
        "/Shooter/Flywheel/FlywheelAppliedOutput", flywheelMotor.getMotorOutputVoltage());
    Logger.recordOutput(
        "/Shooter/Flywheel/FeederAppliedOutput", feederMotor.getMotorOutputVoltage());
  }

  public void setFlywheelVelocity(double velocityRPM) {
    // DO NOTHING
  }

  public double getFlywheelVelocity() {
    return 0.0;
  }

  // to help the kp value from freaking out at low speeds
  public void readjustPID() {
    // DO NOTHING
  }

  /** Set the Flywheel to the specific speed. */
  public void testFlywheelVoltage(double volts) {
    outputVoltage = volts;
    // OK to just divide by 12 because voltage compensation is enabled
    flywheelMotor.set(TalonSRXControlMode.PercentOutput, volts / 12.0);
    if (volts > 1) {
      feederMotor.set(TalonSRXControlMode.PercentOutput, 1);
    } else {
      feederMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }
}
