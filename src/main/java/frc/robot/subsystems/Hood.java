package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    
    private final Servo leftServo;
    private final Servo rightServo;

    public Hood(int leftHoodID, int rightHoodID) {
        this.leftServo = new Servo(leftHoodID);
        this.rightServo = new Servo(rightHoodID);
    }

    public void setHoodAngle(double angle) {
        this.leftServo.setPosition(angle);
        this.rightServo.setPosition(angle);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Servo Position",
            () -> this.leftServo.getPosition(),
            (double d) -> this.setHoodAngle(d)
        );
        builder.addDoubleProperty(
            "Servo Speed", 
            () -> this.leftServo.getSpeed(), 
            null
        );
    }

    public class Commands {
        public Command setHood(DoubleSupplier supplier) {
            return Hood.this.runOnce(
                () -> Hood.this.setHoodAngle(supplier.getAsDouble())
            );
        }
    }
}
