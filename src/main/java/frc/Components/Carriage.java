package frc.Components;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Devices.BinarySensor;
import frc.Devices.Motor.TalonFX;
import frc.lib.util.MotionController;
import frc.lib.util.PDConstant;
import frc.lib.util.PIDController;

public class Carriage extends SubsystemBase {
    Double startPos = null;
    public TalonFX motor;
    public BinarySensor noteSensor;
    MotionController controller = new PIDController(new PDConstant(4, 0, 3.0));
    boolean prepShot = false;
    boolean isFiring = false;
    boolean hasNote = true;

    public void setHasNote(boolean hasNote) {
        this.hasNote = hasNote;
    }

    public boolean isFiring() {
        return isFiring;
    }

    public Carriage(TalonFX motor, BinarySensor noteSensor) {
        this.motor = motor;
        this.noteSensor = noteSensor;
    }

    public void intake() {
        if (!hasNote) {
            motor.setVelocity(0.3 * 360);
        } else {

        }

    }

    public void stop() {
        motor.setVelocity(0);
    }

    public void outTake() {
        hasNote = false;
        motor.setVoltage(-5);
        startPos = null;
    }

    public void prepShot() {
        prepShot = true;
    }

    public void unPrepShot() {
        prepShot = false;
    }

    public void shoot() {
        Command shoot = new Command() {
            @Override
            public void initialize() {
                motor.setVoltage(12);
                startPos = null;
            }

            @Override
            public void execute() {

            }

            @Override
            public void end(boolean interrupted) {

            }
        }.withTimeout(0.25);
        shoot.schedule();
        hasNote = false;

    }

    public boolean hasNote() {
        return hasNote;
    }

    @Override
    public void periodic() {
        if (noteSensor.justEnabled()) {
            startPos = motor.getRevs();
            prepShot = false;
        }
        if (motor.getVoltage() >= 11) {
            isFiring = true;
        } else {
            isFiring = false;
        }

        if (startPos != null) {
            motor.setVoltage(controller.solve(-(motor.getRevs() - (prepShot ? startPos + 4 : startPos + 1.5)), 0.02));
            hasNote = true;
        }
    }
}
