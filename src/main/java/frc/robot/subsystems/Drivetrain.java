package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase{

    TalonSRX leftTalon1;
    TalonSRX leftTalon2;
    TalonSRX leftTalon3;
    TalonSRX rightTalon1;
    TalonSRX rightTalon2;
    TalonSRX rightTalon3;

    public Drivetrain(){
        leftTalon1 = new TalonSRX(1);
        leftTalon2 = new TalonSRX(2);
        leftTalon3 = new TalonSRX(3);
        rightTalon1 = new TalonSRX(6);
        rightTalon2 = new TalonSRX(7);
        rightTalon3 = new TalonSRX(8);
    }

    public void setMotors (double speed, double turn){
        turn *= -1;
        leftTalon1.set(ControlMode.PercentOutput, speed + turn);
        leftTalon2.set(ControlMode.PercentOutput, speed + turn);
        leftTalon3.set(ControlMode.PercentOutput, speed + turn);
        rightTalon1.set(ControlMode.PercentOutput, speed - turn);
        rightTalon2.set(ControlMode.PercentOutput, speed - turn);
        rightTalon3.set(ControlMode.PercentOutput, speed - turn);

        
    }

}
