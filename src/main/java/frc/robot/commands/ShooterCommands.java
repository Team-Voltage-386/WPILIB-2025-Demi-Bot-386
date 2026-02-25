// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;

public class ShooterCommands {

  private ShooterCommands() {}

  /**
   * Standard joystick drive, where X is the forward-backward axis (positive = forward) and Z is the
   * left-right axis (positive = counter-clockwise).
   */
  public static Command constantShooter(Shooter shooter, BooleanSupplier triggerSupplier) {
    return Commands.run(
        () -> {

          // Run or stop
          if (triggerSupplier.getAsBoolean()) {
            shooter.runFlywheel();
          } else {
            shooter.stopFlywheel();
          }
        },
        shooter);
  }
}
