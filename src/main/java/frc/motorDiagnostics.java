package frc;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.Faults;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class motorDiagnostics implements Sendable {
    
    SparkMax m_Motor;
    String m_Name;
    Sendable mSendable;

    double l_maxCurrent = 0;

    public motorDiagnostics(SparkMax motor, String name)
    {
        m_Motor = motor;
        m_Name = name;
    }

    public void publishMotorData()
    {
        SmartDashboard.putData(m_Name, this);

        l_maxCurrent = Math.max(l_maxCurrent, m_Motor.getOutputCurrent());

    }

    private String getFault() {
        String currentFault = "None";
        Faults cFaults = m_Motor.getFaults();

        if(cFaults.can)
            currentFault = "CAN Fault";
        else if(cFaults.escEeprom)
            currentFault = "Esc EE Prom";
        else if(cFaults.firmware)
            currentFault = "Firmware";
        else if(cFaults.gateDriver)
            currentFault = "Gate Driver";
        else if(cFaults.motorType)
            currentFault = "Motor Type";
        else if(cFaults.sensor)
            currentFault = "Sensor Fault";
        else if(cFaults.temperature)
            currentFault = "Temperature";
        else if(cFaults.other)
            currentFault = "Other";
    
        return currentFault;
    }

    private double getMaxCurrent() {
        return l_maxCurrent;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Diagnostics");
        builder.addDoubleProperty("output", m_Motor::get, null);
        builder.addStringProperty("faults", this::getFault, null);
        builder.addBooleanProperty("isFollower", m_Motor::isFollower, null);
        builder.addDoubleProperty("current", m_Motor::getOutputCurrent, null);
        builder.addDoubleProperty("Max Current", this::getMaxCurrent, null);
    }
}
