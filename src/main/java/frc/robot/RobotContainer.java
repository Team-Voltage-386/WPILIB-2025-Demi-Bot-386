// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOTalonSRX;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonSRX;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final LightSubsystem ledLights = new LightSubsystem();

  private final Drive drive;
  // Subsystems
  private Flywheel flywheel;
  private double flywheelVoltage = 0.0;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    ledLights.setToColor(0, 200, 0, 0);
    ledLights.setToColor(1, 0, 0, 200);
    ledLights.setToColor(2, 0, 200, 0);
    ledLights.setToColor(3, 200, 200, 0);
    ledLights.setToColor(3, 0, 200, 200);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOTalonSRX(), new GyroIOPigeon2());
        flywheel = new Flywheel(new FlywheelIOTalonSRX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        flywheel = new Flywheel(new FlywheelIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal tank drive
    drive.setDefaultCommand(
        DriveCommands.tankDrive(
            drive, () -> -controller.getLeftY(), () -> -controller.getRightY()));

    controller
        .a()
        .onTrue(
            (new InstantCommand(
                () -> {
                  flywheelVoltage = 6.0;
                  Logger.recordOutput("/Shooter/Flywheel/ButtonSetpoint", flywheelVoltage);
                  ledLights.changeAllLEDColor(255, 0, 0);
                  System.out.println("Set voltage A: 6.0 Volts");
                })));

    controller
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  flywheelVoltage = 8.0;
                  Logger.recordOutput("/Shooter/Flywheel/ButtonSetpoint", flywheelVoltage);
                  ledLights.changeAllLEDColor(0, 255, 0);
                  System.out.println("Set voltage B: 8.0 Volts");
                }));
    controller
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  flywheelVoltage = 10.0;
                  Logger.recordOutput("/Shooter/Flywheel/ButtonSetpoint", flywheelVoltage);
                  ledLights.changeAllLEDColor(0, 0, 255);
                  System.out.println("Set voltage X: 10.0 Volts");
                }));

    controller
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  flywheelVoltage = 12.0;
                  Logger.recordOutput("/Shooter/Flywheel/ButtonSetpoint", flywheelVoltage);
                  ledLights.changeAllLEDColor(255, 255, 0);
                  System.out.println("Set voltage Y: 12.0 Volts");
                }));

    controller
        .rightTrigger()
        .whileTrue(new InstantCommand(() -> flywheel.io.testFlywheelVoltage(flywheelVoltage)));
    controller
        .rightTrigger()
        .onFalse(new InstantCommand(() -> flywheel.io.testFlywheelVoltage(0.0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
