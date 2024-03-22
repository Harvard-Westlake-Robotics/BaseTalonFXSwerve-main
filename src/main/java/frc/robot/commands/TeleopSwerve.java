package frc.robot.commands;

import frc.Components.Carriage;
import frc.Components.Elevator;
import frc.Components.Shooter;
import frc.Components.Auto.AutoAim;
import frc.Devices.Motor.TalonFX;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private Shooter shooter;
    private Elevator elevator;
    private TalonFX intake;
    private Carriage carriage;
    private AutoAim autoAim;
    private boolean isFieldOriented;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, Elevator elevator, Shooter shooter,
            TalonFX intake, Carriage carriage, AutoAim autoAim) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.carriage = carriage;
        this.intake = intake;
        this.elevator = elevator;
        this.shooter = shooter;
        this.autoAim = autoAim;
        isFieldOriented = true;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband)
                + autoAim.getCorrection();
        isFieldOriented = SmartDashboard.getBoolean("Field-Oriented Control", true);

        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                isFieldOriented,
                true);

        if (carriage.hasNote()) {
            SmartDashboard.putString("Carriage", "Its inside of me");
        } else {
            SmartDashboard.putString("Carriage", "Out Daddy");
        }
        if (shooter.isSpinning()) {
            SmartDashboard.putString("Shooter Spinning", "Shooter Is Spinning");
        } else {
            SmartDashboard.putString("Shooter Spinning", "Shooter Not Spinning");
        }
    }
}