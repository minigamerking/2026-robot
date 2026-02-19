// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Shooter shooter = new Shooter(ShooterConstants.TOPSHOOTERMOTORPORT, ShooterConstants.BOTTOMSHOOTERMOTORPORT);
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final Hood hood = new Hood(ShooterConstants.LEFTLINEARSERVOPORT, ShooterConstants.RIGHTLINEARSERVOPORT);
  //private final Turret turret = new Turret(TurretConstants.TURRETMOTORPORT, 1, TurretConstants.ENCODERREVERSED);

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVERCONTROLLERPORT);

  public RobotContainer() {
    SmartDashboard.putData("Reset Headings", swerve.resetSwerveHeadings());
    SmartDashboard.putData("Swerve", swerve);
    SmartDashboard.putData("Hood", hood);
    SmartDashboard.putData("Shooter", shooter);
    //SmartDashboard.putData("Turret", turret);
    // Configure the controller bindings
    configureBindings();
  }

  private void configureBindings() {
    // Sets the shooter to always respond to the trigger
    this.driverController.a().whileTrue(this.shooter.commands.shoot());//.alongWith(this.shooter.commands.increaseSpeed(() -> 0.025).onlyWhile(() -> true)));

    /*swerve.setDefaultCommand(new SwerveDriveCommand(
        () -> -this.driverController.getLeftY(),
        () -> -this.driverController.getLeftX(),
        () -> this.driverController.getRightX(),
        swerve
    ));*/
  }
}
