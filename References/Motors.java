// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.References;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/** List of how to call common motors in FRC and configure common motor settings  */

public class Motors {

/* Rev Spark max and Brushed Motor by CAN */
private final CANSparkMax brushedCanSparkMax= new CANSparkMax(0, MotorType.kBrushed);
// how to invert it 
brushedCanSparkMax.setInverted(true);    

/* Rev Spark max and Brushed Motor by PWM */
private final PWMSparkMax brushedPWMSparkMax= new PWMSparkMax(0);
// how to invert it 
brushedPWMSparkMax.setInverted(true);    


/* Rev Spark max and Rev Neo550 or Neo Motors by CAN */
private final CANSparkMax neoCanSparkMax= new CANSparkMax(0, MotorType.kBrushless);
// how to invert it 
neoCanSparkMax.setInverted(true);

/* Rev Spark max and Rev Neo550 or Neo Motors by CAN */
private final PWMSparkMax neoPWMSparkMax= new PWMSparkMax(0);
// how to invert it 
neoPWMSparkMax.setInverted(true);

/* Rev Spark Vortex by CAN */
private final CANSparkFlex votexCanSparkMax= new CANSparkFlex(0, MotorType.kBrushless);
// how to invert it 
vortexCanSparkMax.setInverted(true);

/* CTRE VictorSP and Bushed motor by PWM */
private final VictorSP brushedVictorSP = new VictorSP(0);
// how to invert it 
brushedVictorSP.setInverted(true);


/* CTRE TalonSRX and Bushed motor by can */
private final TalonSRX brushedTalonSRX = new TalonS;
// how to invert it 
brushedVictorSP.setInverted(true);





}
