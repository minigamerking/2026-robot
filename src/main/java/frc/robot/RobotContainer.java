// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Shooter shooter = new Shooter(ShooterConstants.SHOOTERMOTORONEID, ShooterConstants.SHOOTERMOTORTWOID);

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVERCONTROLLERPORT);

  public RobotContainer() {
    // Configure the controller bindings
    configureBindings();
  }

  private void configureBindings() {
    // Sets the shooter to always respond to the trigger
    this.shooter.setDefaultCommand(
      this.shooter.commands.shoot(() -> this.driverController.getRightTriggerAxis())
    );
    //this.driverController.rightTrigger().whileTrue(this.shooter.commands.shoot(this.driverController.getRightTriggerAxis()));
  }
}
