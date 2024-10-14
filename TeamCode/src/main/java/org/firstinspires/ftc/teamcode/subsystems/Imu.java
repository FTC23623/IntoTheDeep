package org.firstinspires.ftc.teamcode.subsystems;

public interface Imu {
    public boolean Connected();
    public boolean Calibrating();
    public void Close();
    public void ResetYaw();
    public double GetYaw();
    public void SetYawOffset(double offset);
}
