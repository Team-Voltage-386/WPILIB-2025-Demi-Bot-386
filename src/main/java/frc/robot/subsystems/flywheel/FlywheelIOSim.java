package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;

public class FlywheelIOSim implements FlywheelIO {

  private AngularVelocity flywheelSpeed = RPM.zero();
  private double outputVoltage = 0;

  public FlywheelIOSim() {}

  public void updateInputs(FlywheelIO.FlywheelIOInputs inputs) {
    inputs.connected = true;
    inputs.flywheelSpeed = flywheelSpeed;
    // assuming 12 meters per second at 6000 RPM
    inputs.shotSpeed = MetersPerSecond.of(flywheelSpeed.in(RPM) / 6000 * 12);

    // allows to add logic later to gradually increment applied voltage until it reaches setpoint
    // only here to visualize in simulation
    if (outputVoltage >= 0) {
      if (inputs.outputVoltage < outputVoltage) {
        inputs.outputVoltage += 0.1;
      } else {
        inputs.outputVoltage -= 0.1;
      }
    } else {
      if (inputs.outputVoltage > outputVoltage) {
        inputs.outputVoltage -= 0.1;
      } else {
        inputs.outputVoltage += 0.1;
      }
    }

    Logger.recordOutput("/Shooter/Flywheel/VoltageSetpoint", outputVoltage);
    Logger.recordOutput("/Shooter/Flywheel/AppliedOutput", inputs.outputVoltage);
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    flywheelSpeed = RPM.of(rpm);
    Logger.recordOutput("/Shooter/Flywheel/VelocitySetpoint", rpm);
  }

  @Override
  public double getFlywheelVelocity() {
    return flywheelSpeed.in(RPM);
  }

  /** Set the Flywheel to the specific speed. */
  public void testFlywheelVoltage(double volts) {
    outputVoltage = volts;
  }
}
