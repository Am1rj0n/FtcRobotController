package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrive {

    private final DcMotor fl, fr, bl, br;

    public MecanumDrive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    public void drive(double x, double y, double turn) {
        double flP = y + x + turn;
        double blP = y - x + turn;
        double frP = y - x - turn;
        double brP = y + x - turn;

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

        fl.setPower(flP / denom);
        bl.setPower(blP / denom);
        fr.setPower(frP / denom);
        br.setPower(brP / denom);
    }
}
