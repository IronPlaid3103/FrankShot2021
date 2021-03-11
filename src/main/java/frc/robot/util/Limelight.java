//Borrowed heavily from https://github.com/Team2791/Robot_2018/blob/master/src/org/usfirst/frc/team2791/robot/util/Limelight.java

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Limelight {
    private NetworkTable _table;
    private NetworkTableEntry _camMode, _ledMode, _pipeline, _tx, _ty, _ta, _tv, _ts, _tl;

    public Limelight() {
        _table = NetworkTableInstance.getDefault().getTable("limelight");

        _tx = _table.getEntry("tx");
        _ty = _table.getEntry("ty");
        _ta = _table.getEntry("ta");
        _tv = _table.getEntry("tv");
        _ts = _table.getEntry("ts");
        _tl = _table.getEntry("tl");

        _ledMode = _table.getEntry("ledMode");
        _camMode = _table.getEntry("camMode");

        _pipeline = _table.getEntry("pipeline");
    }

    public double getHorizontalOffset() {
        return _tx.getDouble(0.0);
    }

    public double getVerticalOffset() {
        return _ty.getDouble(0.0);
    }

    public double getTargetArea() {
        return _ta.getDouble(0.0);
    }

    public double getTargetSkew() {
        return _ts.getDouble(0.0);
    }

    public double getLatency() {
        return _tl.getDouble(0.0);
    }

    public boolean isTargetValid() {
    	return (_tv.getDouble(0.0) == 1.0);
    }

    public double getDistance() {
        double targetAngle = Math.toRadians(Constants.LimelightConstants.mountingAngle + this.getVerticalOffset());
        SmartDashboard.putNumber("Target Angle", targetAngle);
        double targetHeight = Constants.FieldConstants.targetHeight - Constants.LimelightConstants.mountingHeight;
        SmartDashboard.putNumber("Target Height", targetHeight);
        return targetHeight / Math.tan(Math.toRadians(targetAngle));
    }

    public void setLed(String mode) {
        switch(mode) {
            case "on":
                _ledMode.setNumber(0); break;
            case "off":
                _ledMode.setNumber(1); break;
            case "blink":
                _ledMode.setNumber(2); break;
            default:
                _ledMode.setNumber(0);
        }
    }

    public void setCameraMode(String mode) {
        switch(mode) {
            case "vision":
                _camMode.setNumber(0); break;
            case "driver":
                _camMode.setNumber(1); break;
            default:
                _camMode.setNumber(0);
        }
    }

    public void setPipeline(int pipeline) {
        _pipeline.setNumber(pipeline);
    }
}
