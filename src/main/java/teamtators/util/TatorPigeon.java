package teamtators.util;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.networktables.NetworkTableEntry;

public class TatorPigeon extends Pigeon2 {
    private Timer timer;
    private double last;
    private double offset = 0;
    public NetworkTableEntry gyroOffset;

    public TatorPigeon(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
        timer = new Timer();
    }

    public TatorPigeon(int deviceNumber) {
        super(deviceNumber);
        timer = new Timer();
    }

    public void zero() {
        offset = getYaw() + 90;
    }

    public double getYawD() {
        return 270 - getYaw() + offset;
    }

    private void updateYaw() {
        last = getYawD();
    }


    public double getYawContinuous() {
        return -getYaw() + offset - 90;
    }

    public boolean isConnected() {
        return true;
    }

    public void setCurrentAngle(double angle) {
        zero();
        offset += angle;
    }
}
