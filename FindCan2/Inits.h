void canDetectInit() {
    for (int i = 0; i < IR_END; i++) {
        obj[i].active = false;
    }
}

void canStorageInit() {
    for (int i = 0; i < canCapacity; i++) {
        cans[i].next = i + 1;
        cans[i].ts = 0;
        if (i == canCapacity - 1) {
            cans[i].next = -1;
        }
    }
}

void wayPtInit() {
    wayPts[0].pos.x = GOAL_LINE / 2;
    wayPts[0].pos.y = 0;
    wayPts[1].pos.x = MAX_X;
    wayPts[1].pos.y = MAX_Y / 2;
    wayPts[2].pos.x = MIN_X;
    wayPts[2].pos.y = MAX_Y / 2;
    wayPts[3].pos.x = MIN_X;
    wayPts[3].pos.y = MIN_Y / 2;
    wayPts[4].pos.x = MAX_X;
    wayPts[4].pos.y = MIN_Y / 2;
    wayPts[5].pos.x = MIN_X;
    wayPts[5].pos.y = MAX_Y;
    wayPts[6].pos.x = MAX_X;
    wayPts[6].pos.y = MAX_Y;
    wayPts[7].pos.x = MIN_X;
    wayPts[7].pos.y = MIN_Y;
    wayPts[8].pos.x = MAX_X;
    wayPts[8].pos.y = MIN_Y;
}

void sensorOffsetInit() {
    // Sensor offsets from robot center
    sensors[IRL].offset.x = 0;
    sensors[IRL].offset.y = 8.0;
    sensors[IRL].angle = 90 * PI / 180;
    sensors[IRFL].offset.x = 4.1;
    sensors[IRFL].offset.y = 6.2;
    sensors[IRFL].angle = 45 * PI / 180;
    sensors[IRF].offset.x = 7.3;
    sensors[IRF].offset.y = 1.2;
    sensors[IRF].angle = 0 * PI / 180;
    sensors[IRFR].offset.x = 5.7;
    sensors[IRFR].offset.y = -4.8;
    sensors[IRFR].angle = -45 * PI / 180;
    sensors[IRR].offset.x = 0;
    sensors[IRR].offset.y = -8.0;
    sensors[IRR].angle = -90 * PI / 180;
}

void robotBaseInit() {
    //Establish PID gains
    RobotBase.setPID(10, 5, 0);

    //Set allowed accel for wheel velocity targets (cm/s/s)
    RobotBase.setAccel(50);

    //Set max velocity and turn rate
    RobotBase.setVelocityRange(20.0, 0, 1.0);
    RobotBase.setTurnRange(2.0, 0.01, 0.2);

    //set motor output ranges - works both positive and negative
    //Max, dead zone, min
    // -deadZone > X < deadZone : X = 0
    // X < min : X = min
    // X > max : x = max
    RobotBase.setOutputRange(350, 5, 70);

    //set ticks per desired distance unit
    RobotBase.setTicksPerUnit(71.65267); //units of cm

    //set the wheelbase of in desired distance unit
    RobotBase.setWidth(17.4); //units of cm

    RobotBase.setNavThresh(2, 0.04);
    RobotBase.setOdomPeriod(10);
    RobotBase.setNavPeriod(10);
    RobotBase.setIRPeriod(40);
    RobotBase.setIRFilter(0.7);
    RobotBase.setIRSamples(1);
}

void lcdInit() {
    lcd.setMCPType(LTI_TYPE_MCP23017);
    // set up the LCD's number of columns and rows: 
    lcd.begin(16, 2);
    lcd.setBacklight(WHITE);
}

void sonarInit() {
    pinMode(SNR_RX, OUTPUT);
    pinMode(SNR_PW, INPUT);
    attachInterrupt(SNR_PW, readSonarPulse, CHANGE);
}

void bumperInit() {
    pinMode(IRB_FR, INPUT);
    pinMode(IRB_F, INPUT);
    pinMode(IRB_FL, INPUT);
}

void gripperInit() {
    servoG.attach(SERVO_G);
}

