package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterSpeed extends Command {
    private final Shooter shooter;
    private final DoubleSupplier supplier;

    public ShooterSpeed(Shooter shooter, DoubleSupplier supplier) {
        this.shooter = shooter;
        this.supplier = supplier;
    }

    @Override
    public void execute() {
        this.shooter.setSpeed(this.supplier.getAsDouble());
    }
}
