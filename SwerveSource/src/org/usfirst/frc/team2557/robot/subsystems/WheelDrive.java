package org.usfirst.frc.team2557.robot.subsystems;

import org.usfirst.frc.team2557.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class WheelDrive extends Subsystem {
	private WPI_TalonSRX angleMotor;
	private WPI_TalonSRX speedMotor;
	private PIDController pidController;

	public WheelDrive (int angleMotor, int speedMotor, int encoder) {
	    this.angleMotor = new WPI_TalonSRX(angleMotor);
	    this.speedMotor = new WPI_TalonSRX(speedMotor);
	    pidController = new PIDController (1, 0, 0, new AnalogInput (encoder), this.angleMotor);

	    pidController.setOutputRange(-1, 1);
	    pidController.setContinuous();
	    pidController.enable();
	}
	private final double MAX_VOLTS = 1;
	
	public void drive(double speed, double angle) {
	    this.speedMotor.set (speed);
	    double setpoint = angle*(MAX_VOLTS * 0.5)+(MAX_VOLTS * 0.5);
	    if (setpoint < 0) {
	        setpoint = MAX_VOLTS+setpoint;
	    }
	    if (setpoint > MAX_VOLTS){
	        setpoint = setpoint-MAX_VOLTS;
	    }

	    pidController.setSetpoint(setpoint);
	}
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

