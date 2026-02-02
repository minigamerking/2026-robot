package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;

    public Intake(int intakeMotorPort) {
        this.intakeMotor = new TalonFX(intakeMotorPort);
    }

    public void intake(double speed) {
        this.intakeMotor.set(speed);
    }

    public class Commands {
        public Command intake(double speed) {
            return Intake.this.runOnce(
                () -> Intake.this.intake(speed)
            );
        }
    }
}
