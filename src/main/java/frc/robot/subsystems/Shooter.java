// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.FunctionUtilities;

public class Shooter extends SubsystemBase {

  private final TalonFX topShooterMotor;
  private final TalonFX bottomShooterMotor;

  public final Shooter.Commands commands = new Commands();

  private final VelocityVoltage topRequest = new VelocityVoltage(0);
  private final VelocityVoltage bottomRequest = new VelocityVoltage(0);
  
  private double maxSpeed = 0.15;
  
  private double topTargetRPS = 0;
  private double bottomTargetRPS = 0;

  public Shooter(int topShooterID, int bottomShooterID) {
    this.topShooterMotor = new TalonFX(topShooterID);
    this.topShooterMotor.getConfigurator().apply(getTopMotorConfig());

    this.bottomShooterMotor = new TalonFX(bottomShooterID);
    this.bottomShooterMotor.getConfigurator().apply(getBottomMotorConfig());
  }

  private TalonFXConfiguration getTopMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kV = ShooterConstants.TOPMOTORKV;
    config.Slot0.kA = ShooterConstants.TOPMOTORKA;
    config.Slot0.kP = ShooterConstants.TOPMOTORKP;

    return config;
  }

  private TalonFXConfiguration getBottomMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kV = ShooterConstants.BOTTOMMOTORKV;
    config.Slot0.kA = ShooterConstants.BOTTOMMOTORKA;
    config.Slot0.kP = ShooterConstants.BOTTOMMOTORKP;

    return config;
  }

  public void setSpeed(double topSpeed, double bottomSpeed) {
    this.topTargetRPS = topSpeed * 100;
    this.bottomTargetRPS = bottomSpeed * 100;
  }

  @Override
  public void periodic() {
    this.topShooterMotor.setControl(topRequest.withVelocity(this.topTargetRPS));
    this.bottomShooterMotor.setControl(bottomRequest.withVelocity(this.bottomTargetRPS));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
      "Shooter Speed", 
      () -> this.maxSpeed, 
      (double d) -> this.maxSpeed = d
    );
    builder.addDoubleArrayProperty(
      "Motor RPS", 
      () -> new double[]{this.topShooterMotor.getRotorVelocity().getValueAsDouble(), this.bottomShooterMotor.getRotorVelocity().getValueAsDouble()}, 
      null
    );
    builder.addDoubleArrayProperty(
      "Motor Current Draw", 
      () -> new double[]{this.topShooterMotor.getSupplyCurrent().getValueAsDouble(), this.bottomShooterMotor.getSupplyCurrent().getValueAsDouble()}, 
      null
    );
  }

  public class Commands {
    public Command shoot(DoubleSupplier topSupplier, DoubleSupplier bottomSupplier) {
      return Shooter.this.run(
        () -> Shooter.this.setSpeed(topSupplier.getAsDouble(), bottomSupplier.getAsDouble())
      ).finallyDo(
        () -> Shooter.this.setSpeed(0, 0)
      );
    }

    public Command shoot() {
      return Shooter.this.run(
        () -> Shooter.this.commands.shoot(() -> FunctionUtilities.applyClamp(Shooter.this.maxSpeed * 3, 0, 1), () -> Shooter.this.maxSpeed)
      ).finallyDo(
        () -> Shooter.this.commands.stopShooting()
      );
    }

    public Command stopShooting() {
      return Shooter.this.runOnce(
        () -> Shooter.this.setSpeed(0, 0)
      );
    }
  }
}
