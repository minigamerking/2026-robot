// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  //private final Shooter shooter = new Shooter(ShooterConstants.SHOOTERMOTORONEPORT, ShooterConstants.SHOOTERMOTORTWOPORT);
  private final SwerveSubsystem swerve = new SwerveSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVERCONTROLLERPORT);

  public RobotContainer() {
    SmartDashboard.putData("Reset Headings", swerve.resetSwerveHeadings());
    SmartDashboard.putData("Swerve", swerve);
    // Configure the controller bindings
    configureBindings();
  }

  private void configureBindings() {
    // Sets the shooter to always respond to the trigger
    /*this.shooter.setDefaultCommand(
      this.shooter.commands.shoot(() -> this.driverController.getRightTriggerAxis())
    );*/

    swerve.setDefaultCommand(new SwerveDriveCommand(
        () -> -this.driverController.getLeftY(),
        () -> -this.driverController.getLeftX(),
        () -> this.driverController.getRightX(),
        swerve
    ));
  }
}
