package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

@Autonomous
public class OfficialAutonomousBlue extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Motor1 = null;
    private DcMotor Motor2 = null;
    private DcMotor Motor3 = null;
    private DcMotor Motor4 = null;
    private ColorSensor colorSensor;
    private Servo SensorServo =null;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;
    private boolean isScanDone = false;
    
    @Override
    public void runOpMode () {
        boolean bLedOn = true;
        boolean isOpModeActive = true;
        
        //Ititialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
        Motor4 = hardwareMap.get(DcMotor.class, "Motor4");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        SensorServo  = hardwareMap.get(Servo.class, "SensorServo");
        
        Motor1.setDirection(DcMotor.Direction.FORWARD);
        Motor2.setDirection(DcMotor.Direction.FORWARD);
        Motor3.setDirection(DcMotor.Direction.FORWARD);
        Motor4.setDirection(DcMotor.Direction.FORWARD);
        colorSensor.enableLed(bLedOn);
       
        //Wait for the game to start
        waitForStart();
        runtime.reset();
        
        //Gradual movement down so that the sensor doesn't get damaged due to momentum
        SensorServo.setPosition(0.5);
        while ((runtime.seconds() <= 1)&&(opModeIsActive())&&(isOpModeActive == true)) {
        }
        SensorServo.setPosition(0.3);
        while ((runtime.seconds() <= 10)&&(opModeIsActive())&&(isOpModeActive == true)&&(isScanDone == false)) {
            if(colorSensor.red()>0&&colorSensor.blue()==0){ //if red
                //pivot robot 
                while(runtime.seconds() <=1.7){ 
                Motor1.setPower(-0.1);
                Motor2.setPower(-0.1);
                Motor3.setPower(-0.1);
                Motor4.setPower(-0.1);
                }
                Motor1.setPower(0);
                Motor2.setPower(0);
                Motor3.setPower(0);
                Motor4.setPower(0);
                
                isScanDone = true;
            }
            else if(colorSensor.blue()>0&&colorSensor.red()==0){
                while(runtime.seconds() <=1.7){
                Motor1.setPower(0.1);
                Motor2.setPower(0.1);
                Motor3.setPower(0.1);
                Motor4.setPower(0.1);
                }
                Motor1.setPower(0);
                Motor2.setPower(0);
                Motor3.setPower(0);
                Motor4.setPower(0);

            }
        }
    }
}
