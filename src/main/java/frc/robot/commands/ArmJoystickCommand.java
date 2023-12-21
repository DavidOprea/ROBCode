package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Potentiometer;

public class ArmJoystickCommand extends CommandBase {
    
    Arm arm = new Arm();
    Potentiometer pot = new Potentiometer();
    Supplier<Double> raise;
    double targetPos;
    double kDt;
    TrapezoidProfile.Constraints m_constraints;
    ProfiledPIDController m_controller;

    public ArmJoystickCommand(Arm arm, Potentiometer pot, Supplier<Double> raise) {
        this.arm = arm;
        this.pot = pot;
        this.raise = raise;
        addRequirements(arm);
    }

    public void initialize() {
    }

    @Override 
    public void execute(){
    }
@Override 
public void end(boolean interrupted){
}

@Override
public boolean isFinished(){
return false;
}   

    
}