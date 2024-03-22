package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.Components.Auto.Position;
import frc.Core.FieldData;
import frc.Core.Time;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.LinkedList;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Devices.Imu;
import frc.Devices.LimeLight;
import frc.lib.util.Vector2;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Imu gyro;
    private Field2d field = new Field2d();

    public Swerve() {
        gyro = new Imu(Constants.Swerve.pigeonID);
        limeLight = new LimeLight(Constants.Swerve.shooterLimeLightID);
        gyro.setYaw(0);
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", field);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void fromChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getRobotVelocity() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(this.getModuleStates());
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    private LimeLight limeLight;
    boolean hasGottenLimeLightFrame = false;
    double lastLimelightFrameTime = Double.NEGATIVE_INFINITY;
    Position lastLimelightFrameOffset = new Position(0, new Vector2(0, 0));
    LinkedList<Position> positionHistory = new LinkedList<>();
    Pose2d poseChangeSinceLimeLightFrame;
    Pose2d lastLimeLightPose;

    private boolean isRationalLimelightFrame() {
        final boolean isAllZero = limeLight.getRobotX() == 0 && limeLight.getRobotY() == 0
                && limeLight.getRobotZ() == 0;
        return limeLight.botPoseChanged() && !isAllZero;
    }

    public Position predictedPositionAtLastLimelightFrame() {
        long currentTimeSinceEpoch = System.currentTimeMillis();
        long receiveTimeSinceEpoch = limeLight.getLastReceiveTime();
        double latencyImageTakenToNow = limeLight.getRobotLatency() / 1000;

        int predictedPositionIndex = (int) (latencyImageTakenToNow / 0.02);

        if (predictedPositionIndex >= positionHistory.size()) {
            return null;
        }

        return positionHistory.get(predictedPositionIndex);

    }

    @Override
    public void periodic() {
        field.setRobotPose(getPose());
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
        positionHistory.add(0,
                new Position(getPose().getRotation().getDegrees(),
                        new Vector2(getPose().getX(), getPose().getY())));

        // makes sure position history doesn't get too long
        if (positionHistory.size() > 5 / 0.02) {
            positionHistory.removeLast();
        }
        if (FieldData.getIsTeleop()) {
            if (isRationalLimelightFrame()) {
                hasGottenLimeLightFrame = true;
                final double timeSinceLastFrame = Time.getTimeSincePower() -
                        lastLimelightFrameTime;
                lastLimelightFrameTime = Time.getTimeSincePower();
                final Position limelightPositionAtFrame = limeLight.getRobotPosition();
                Position predictedPositionAtFrame = predictedPositionAtLastLimelightFrame();
                if (predictedPositionAtFrame == null && positionHistory.size() >= 1)
                    predictedPositionAtFrame = positionHistory.getFirst();

                Position adjustedPosition;
                // if the time since the last reading is more than 20 ms, we scrub the
                // extrapolated position from the drive
                if (timeSinceLastFrame < 20) {
                    adjustedPosition = limelightPositionAtFrame.combine(predictedPositionAtFrame,
                            0.5);
                } else {
                    adjustedPosition = limelightPositionAtFrame;
                }
                if (predictedPositionAtFrame != null) {
                    Position offsetPosition = adjustedPosition.difference(predictedPositionAtFrame);
                    positionHistory.replaceAll(e -> e.add(offsetPosition));
                    System.out.println("Updated based on limelight frame");
                }
            }
            if (limeLight.botPoseChanged() && hasGottenLimeLightFrame && positionHistory.size() >= 1) {
                setPose(positionHistory.getFirst().asPose2d());
            }
        }
    }
}