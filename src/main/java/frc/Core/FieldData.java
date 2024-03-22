package frc.Core;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class FieldData extends SubsystemBase {
    public static boolean getIsRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public static boolean getIsTeleop() {
        return DriverStation.isTeleopEnabled();
    }

    public static boolean getIsAuto() {
        return DriverStation.isAutonomousEnabled();
    }

    public static boolean getIsTest() {
        return DriverStation.isTestEnabled();
    }

    public String getMatchType() {
        if (DriverStation.getMatchType().equals(MatchType.Qualification)) {
            return "Qualification";
        }
        if (DriverStation.getMatchType().equals(MatchType.Elimination)) {
            return "Elimination";
        }
        if (DriverStation.getMatchType().equals(MatchType.Practice)) {
            return "Practice";
        } else {
            return "None";
        }

    }

    public String getMatchTime() {
        int rawTime = (int) DriverStation.getMatchTime();
        int timeMinutes = (int) (rawTime / 60);
        int timeSeconds = rawTime % 60;
        if (timeSeconds < 10) {
            return "0" + timeMinutes + ":" + "0" + timeSeconds;
        }
        return "0" + timeMinutes + ":" + timeSeconds;
    }

    public String getMatchPeriod() {
        if (getIsTeleop()) {
            return "Teleop";
        }
        if (getIsAuto()) {
            return "Autonomous";
        }
        if (getIsTest()) {
            return "Test";
        } else {
            return "None";
        }
    }

    public void periodic() {
        String matchPeriod = getMatchPeriod();
        SmartDashboard.putString("Remaining Match Time", getMatchTime());
        SmartDashboard.putString("Match Period", getMatchPeriod());
        SmartDashboard.putString("Match Type", getMatchType());
    }

}
