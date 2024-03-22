package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.Components.Carriage;
import frc.Components.Elevator;
import frc.Components.Shooter;
import frc.Components.Auto.AutoAim;
import frc.Core.FieldData;
import frc.Devices.Motor.TalonFX;
import frc.lib.util.BetterPS4;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final BetterPS4 driver = new BetterPS4(0);
    private final Joystick joystick = SubsystemInit.joystick();

    /* Subsystems */

    private final Elevator elevator = SubsystemInit.elevator();
    private final Shooter shooter = SubsystemInit.shooter();
    private final TalonFX intake = SubsystemInit.intake();
    private final Carriage carriage = SubsystemInit.carriage(SubsystemInit.intakeSensor());
    public final Swerve s_Swerve = new Swerve();
    private final AutoAim autoAim = new AutoAim(s_Swerve);

    /* Driver Buttons */
    private final JoystickButton moveElevator = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton intakeButton = new JoystickButton(driver, PS4Controller.Button.kL2.value);
    private final JoystickButton outTakeButton = new JoystickButton(driver, PS4Controller.Button.kCross.value);
    private final JoystickButton shooterButton = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    private final JoystickButton fireButton = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final POVButton moveElevatorDownRaw = new POVButton(joystick, 180);
    private final POVButton moveElevatorUpRaw = new POVButton(joystick, 0);
    private final JoystickButton climbDownHard = new JoystickButton(joystick, 3);
    private final JoystickButton moveToClimb = new JoystickButton(joystick, 5);
    private final JoystickButton stretchElevator = new JoystickButton(joystick, 6);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(

                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX(),
                        elevator, shooter, intake, carriage, autoAim));

        AutoBuilder.configureHolonomic(s_Swerve::getPose, s_Swerve::setPose, s_Swerve::getRobotVelocity,
                s_Swerve::fromChassisSpeeds,
                Constants.AutoConstants.getPathFollowerConfig(),
                FieldData::getIsRed, s_Swerve);

        // Configure the button bindings
        // System.out.println("im here");
        configureButtonBindings();

        Constants.AutoConstants.initializeAutonomous(intake, elevator, shooter, carriage, s_Swerve);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        moveElevatorDownRaw.whileTrue(new Command() {
            public void execute() {
                elevator.moveRaw(-0.2 * 360);
            }
        }).onFalse(new Command() {
            public void execute() {
                elevator.moveRaw(0);
            }
        }.withTimeout(0.02));
        moveElevatorUpRaw.whileTrue(new Command() {
            @Override
            public void execute() {
                elevator.moveRaw(0.2 * 360);
            }
        }).onFalse(new Command() {
            @Override
            public void execute() {
                elevator.moveRaw(0);
            }
        }.withTimeout(0.02));
        /* Driver Buttons */
        intakeButton.whileTrue(new Command() {
            @Override
            public void execute() {
                if (carriage.hasNote() || carriage.noteSensor.justEnabled()) {
                    carriage.stop();
                    intake.setVelocity(0);
                } else if (!carriage.hasNote() && elevator.isDown()) {
                    intake.setVelocity(0.3 * 360);
                    carriage.intake();
                }
            }
        }).onFalse(new InstantCommand(() -> {
            carriage.stop();
            intake.setVelocity(0);
        }));
        moveElevator.onTrue(new InstantCommand(() -> {
            if (elevator.isDown())
                elevator.moveUp();
            else
                elevator.moveDown();
        }));

        moveToClimb.onTrue(new InstantCommand(() -> elevator.moveToClimb()));
        climbDownHard.onTrue(new InstantCommand(() -> elevator.climbDown()));
        stretchElevator.onTrue(new InstantCommand(() -> elevator.stretch()));

        outTakeButton.whileTrue(new InstantCommand(() -> {
            carriage.outTake();
            intake.setVoltage(-12);
        })).onFalse(new InstantCommand(() -> {
            intake.setVoltage(0);
            carriage.stop();
        }));
        shooterButton.onTrue(new InstantCommand(() -> {
            if (elevator.isDown()) {
                shooter.toggleSpinning();
                if (shooter.isSpinning()) {
                    carriage.prepShot();
                    autoAim.setIsAutoAimOn(true);
                } else {
                    carriage.unPrepShot();
                    autoAim.setIsAutoAimOn(false);
                }
            }
        }));
        fireButton.onTrue(new InstantCommand(() -> {
            if (shooter.isSpinning()) {
                carriage.shoot();
            } else if (!elevator.isDown()) {
                carriage.outTake();
            }
        })).onFalse(new InstantCommand(() -> {
            carriage.stop();
        }));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Constants.AutoConstants.getAutonToRun();
    }
}
