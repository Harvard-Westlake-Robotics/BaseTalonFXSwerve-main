package frc.Components.Auto;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Components.Carriage;
import frc.Components.Elevator;
import frc.Devices.Motor.TalonFX;
import frc.robot.subsystems.Swerve;
import frc.Components.Shooter;

public class AutoCommands {
    public static void initializeAutonomousCommands(TalonFX intake, Elevator elevator, Shooter shooter,
            Carriage carriage,
            Swerve swerve) {
        if (shooter.isSpinning()) {
            shooter.toggleSpinning();
        }
        NamedCommands.registerCommand("intake", getIntakeCommand(intake, carriage));
        NamedCommands.registerCommand("spinShooter",
                getToggleShooterCommand(carriage, shooter));
        NamedCommands.registerCommand("fireShooter",
                getFireShooterCommand(carriage));

    }

    public static Command getIntakeCommand(TalonFX intake, Carriage carriage) {
        return new Command() {
            public void initialize() {
                intake.setVelocity(0.3 * 360);
                carriage.setHasNote(false);
            }

            @Override
            public void execute() {

                if (carriage.noteSensor.justEnabled()) {
                    intake.setVelocity(0);
                    carriage.stop();
                    this.cancel();
                } else {

                    carriage.intake();
                }
            }

            @Override
            public void end(boolean interrupted) {
                intake.setVelocity(0);
                carriage.stop();

            }
        }.withTimeout(4);
    }

    public static Command getToggleShooterCommand(Carriage carriage, Shooter shooter) {
        return new Command() {
            public void initialize() {
                shooter.toggleSpinning();
                carriage.prepShot();
            }

            @Override
            public void execute() {
                if (carriage.isFiring()) {
                    this.cancel();
                }
            }

            @Override
            public void end(boolean interrupted) {
            }
        }.withTimeout(5.5).andThen(new Command() {
            @Override
            public void initialize() {
            }

            @Override
            public void execute() {
            }

            @Override
            public void end(boolean interrupted) {
                shooter.toggleSpinning();
                carriage.unPrepShot();
            }
        }.withTimeout(0.5));
    }

    public static Command getFireShooterCommand(Carriage carriage) {
        return new Command() {
            public void initialize() {
                carriage.shoot();
            }

            @Override
            public void execute() {
            }

            @Override
            public void end(boolean interrupted) {
                carriage.stop();
            }
        }.withTimeout(0.5);
    }

    public static Command getBaseCommand(Object subsystem) {
        return new Command() {
            public void initialize() {

            }

            @Override
            public void execute() {

            }

            @Override
            public void end(boolean interrupted) {

            }
        };
    }
}
