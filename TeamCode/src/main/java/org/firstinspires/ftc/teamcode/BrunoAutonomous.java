package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Autonomous
public class BrunoAutonomous extends DarrenLinearDrive {
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor Motor1 = null;
    //private DcMotor Motor2 = null;
    //private DcMotor Motor3 = null;
    //private DcMotor Motor4 = null;
    
    @Override
    public void runOpMode () {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    
        //Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        //Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        //Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
        //Motor4 = hardwareMap.get(DcMotor.class, "Motor4");
        
        //Motor1.setDirection(DcMotor.Direction.FORWARD);
        //Motor2.setDirection(DcMotor.Direction.REVERSE);
        //Motor3.setDirection(DcMotor.Direction.FORWARD);
        //Motor4.setDirection(DcMotor.Direction.REVERSE);
       
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        // run until the end of the match (driver presses STOP)
        while (runtime.seconds() <= 5.00001 ) {
        
            //Start of Autonomous Period
            //Motor1.setPower(1);
            //Motor2.setPower(1);
            //Motor3.setPower(1);
            //Motor4.setPower(1);
            telemetry.addData("Status", "Run Time: " + (int) runtime.seconds());
            telemetry.update();
        }
        //Motor1.setPower(0);
        //Motor2.setPower(0);
        //Motor3.setPower(0);
        //Motor4.setPower(0);
        
        
    }
}
