package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link SensorMRRangeSensor} illustrates how to use the Modern Robotics
 * Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@Autonomous(name = "TESTEncoder", group = "Sensor")

public class TESTEncoder extends LinearOpMode {
    public DcMotor Motor5 = null;
    private CRServo JewelServo = null;
    private ElapsedTime Timer = new ElapsedTime();

    @Override public void runOpMode() {
        // get a reference to our compass
        Motor5 = hardwareMap.get(DcMotor.class, "Motor5");
        JewelServo = hardwareMap.get(CRServo.class, "JewelServo");
        JewelServo.setDirection(CRServo.Direction.FORWARD);
        
       Motor5.setDirection(DcMotor.Direction.FORWARD);
       Motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       Motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     //Motor5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       
        // wait for the start button to be pressed
        waitForStart();

        
            
           // for(int x = 0; x<300; x+=1){
                
             //   Motor5.setTargetPosition(180);
             
          Timer.reset();
         while(Timer.seconds()<.3&&opModeIsActive()){
             JewelServo.setPower(-0.55);
         }
         JewelServo.setPower(0);
            while(Motor5.getCurrentPosition()>-310&&opModeIsActive()){
                Motor5.setPower(-0.2);
              
            telemetry.addData("Pos", Motor5.getCurrentPosition());
         
            telemetry.update();
             }
             
             Motor5.setPower(0);
             Timer.reset();
             boolean first = false;
             while(Timer.seconds()<5&&opModeIsActive()){
                
                    Motor5.setPower(0);
                
             }
                     
                     while(Timer.seconds()<.1&&opModeIsActive()){
                         
                    
                    JewelServo.setPower(-0.30);
                     }
                     JewelServo.setPower(0);
                    
             
             
        //     Motor5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
             Motor5.setPower(0);
        
             
        //    }
            
           
    }
    //holds motor for parameter 'time' at parameter 'position' 
    
}
