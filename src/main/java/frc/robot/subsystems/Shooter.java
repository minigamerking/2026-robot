// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final TalonFX shooterMotorOne;
  private final TalonFX shooterMotorTwo;

  public final Shooter.Commands commands = new Commands();
  

  public Shooter(int shooterIDOne, int shooterIDTwo) {
    this.shooterMotorOne = new TalonFX(shooterIDOne);
    this.shooterMotorTwo = new TalonFX(shooterIDTwo);
  }

  public void setSpeed(double speed) {
    this.shooterMotorOne.set(speed);
    this.shooterMotorTwo.set(-speed);
  }

  public class Commands {
    public Command shoot(DoubleSupplier supplier) {
      return Shooter.this.runOnce(
        () -> Shooter.this.setSpeed(supplier.getAsDouble())
      );
    }

    private Command shootSpeed(double speed) {
      return Shooter.this.runOnce(
        () -> Shooter.this.setSpeed(speed)
      );
    }

    public Command shoot() {
      return Shooter.this.runOnce(
        () -> this.shootSpeed(1)
      );
    }
  }
}
