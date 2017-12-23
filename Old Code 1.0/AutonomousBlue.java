package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
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
public class AutonomousBlue extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Motor1 = null;
    private DcMotor Motor2 = null;
    private DcMotor Motor3 = null;
    private DcMotor Motor4 = null;
    private ColorSensor colorSensor;
    
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
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor1");
        
        Motor1.setDirection(DcMotor.Direction.REVERSE);
        Motor2.setDirection(DcMotor.Direction.FORWARD);
        Motor3.setDirection(DcMotor.Direction.REVERSE);
        Motor4.setDirection(DcMotor.Direction.FORWARD);
        colorSensor.enableLed(bLedOn);
       
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        // run until the end of the match driver presses stop or end of 30 seconds
        while ((runtime.seconds() <= 2)&&(opModeIsActive())) {
            Motor1.setPower(1);
            Motor2.setPower(1);
            Motor3.setPower(1);
            Motor4.setPower(1);
        }
        while ((runtime.seconds() <= 10)&&(opModeIsActive())&&(isOpModeActive == true)) {
        
            //Drive robot forward
            Motor1.setPower(1);
            Motor2.setPower(1);
            Motor3.setPower(1);
            Motor4.setPower(1);
            
            //Check for team color tape
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Red ", colorSensor.red());
            telemetry.update();
            if(colorSensor.red()>0&&colorSensor.blue()==0){
                telemetry.addData("red", colorSensor.red());
                telemetry.update();
                isOpModeActive = false;
            }
            
            telemetry.addData("Status", "Run Time: " + (int) runtime.seconds());
            telemetry.update();
        }
        //stop motors
        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);
        
        
    }
}
