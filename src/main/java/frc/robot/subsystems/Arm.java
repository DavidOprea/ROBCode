package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.JoystickCMD;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Arm extends SubsystemBase {
    
    TalonSRX armFour;
    TalonSRX armFive;
    Potentiometer pot;
    double val;
   
   // potEncoder raiseEncoder1 = armFour.getEncoder();

   public Arm(){
        this.armFour = new TalonSRX(4);
        this.armFive = new TalonSRX(5);
        pot = new Potentiometer();
   }

    public void manualArm(ProfiledPIDController m_controller, double curPos){
      System.out.println("Pos: " + curPos);
      val = m_controller.calculate(curPos);
      System.out.println("Val: " + val);
      armFour.set(ControlMode.PercentOutput, val);
      armFive.set(ControlMode.PercentOutput, val);
    }

    public void setVal(double val) {
      this.val = val;
    }
}