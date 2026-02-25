// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.util.Units;

/** This drive implementation is for Talon SRXs driving brushed motors (e.g. CIMS) with encoders. */
public class FlywheelIOTalonSRX implements FlywheelIO {
  private static final double ticksPerRevolution = 1440;

  private final TalonSRX flywheelMotor = new TalonSRX(flywheelCanId);

  public FlywheelIOTalonSRX() {
    var config = new TalonSRXConfiguration();
    config.peakCurrentLimit = flywheelCurrentLimit;
    config.continuousCurrentLimit = flywheelCurrentLimit - 15;
    config.peakCurrentDuration = 250;
    config.voltageCompSaturation = 12.0;
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;

    tryUntilOkV5(5, () -> flywheelMotor.configAllSettings(config));

    flywheelMotor.setInverted(flywheelInverted);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad =
        Units.rotationsToRadians(flywheelMotor.getSelectedSensorPosition() / ticksPerRevolution);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(
            flywheelMotor.getSelectedSensorVelocity()
                / ticksPerRevolution
                * 10.0); // Raw units are ticks per 100ms :(
    inputs.appliedVolts = flywheelMotor.getMotorOutputVoltage();
    inputs.currentAmps = new double[] {flywheelMotor.getStatorCurrent(), 0};
  }

  @Override
  public void setVoltage(double volts) {
    // OK to just divide by 12 because voltage compensation is enabled
    flywheelMotor.set(TalonSRXControlMode.PercentOutput, volts / 12.0);
  }

  @Override
  public void setVelocity(double radPerSec, double FFVolts) {
    // OK to just divide FF by 12 because voltage compensation is enabled
    flywheelMotor.set(
        TalonSRXControlMode.Velocity,
        Units.radiansToRotations(radPerSec)
            * ticksPerRevolution
            / 10.0, // Raw units are ticks per 100ms :(
        DemandType.ArbitraryFeedForward,
        FFVolts / 12.0);
  }
}
