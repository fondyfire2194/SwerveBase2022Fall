package frc.robot.Sim;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;


public class CANSparkMaxWithSim extends CANSparkMax {
    private double m_setPoint;

    public CANSparkMaxWithSim(int deviceID, MotorType type) {
		super(deviceID, type);
    }

    public double get() {
        if (RobotBase.isSimulation()) return m_setPoint;
        else return super.get();
    }

    public void set(double speed) {
        if (RobotBase.isSimulation()) m_setPoint = speed;
        super.set(speed);
    }

    public void setVoltage(double voltage) {
        if (RobotBase.isSimulation()) m_setPoint = voltage/RobotController.getBatteryVoltage();
        super.setVoltage(voltage);
    } 

     // Private helper functions
  private static final CANSparkMaxWithSim getConfiguredMotor(int CANBUSID) {
    var motor = new CANSparkMaxWithSim(CANBUSID, MotorType.kBrushless);
    motor.clearFaults();
    motor.restoreFactoryDefaults();
    return motor;
  }
}
