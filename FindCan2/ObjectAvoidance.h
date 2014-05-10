bool obstacleCenter, obstacleLeft, obstacleRight;
double turnAdjust;
double turnFactor = 5;

void nextWayPt() {
    if (wayPt == wayPtsCnt - 1) {
        wayPt = 0;
    } else
        wayPt++;
    restart = true;
}

void evalWayPoint(int dist, float rads) {
    float relX = dist * cos(rads);
    float relY = dist * sin(rads);
    float relXrotated = relX * cos(RobotBase.getTheta())
            - relY * sin(RobotBase.getTheta());
    float relYrotated = relX * sin(RobotBase.getTheta())
            + relY * cos(RobotBase.getTheta());
    float destX = RobotBase.getX() + relXrotated;
    float destY = RobotBase.getY() + relYrotated;

    if (destX < MAX_X && destX > MIN_X && destY < MAX_Y && destY > MIN_Y) {
        RobotBase.driveTo(destX, destY, true);
    } else {
        RobotBase.stop(true);
    }
}

void scanForOpening() {
    lcd.setBacklight(RED);
    lcd.clear();
    lcd.print("Scan for Opening");
    RobotBase.stop(true);
}

void findRoute() {
    float theta = RobotBase.getTheta();
    float curX = RobotBase.getX();
    float curY = RobotBase.getY();
#if DEBUG_ROUTE
    Serial.println();
    Serial.print("** Start Find Route **");
    Serial.print("  ");
    Serial.println(theta);
    #endif
    
    if (theta < 0 && theta >= -1.57) { // pointing NW 
#if DEBUG_ROUTE
            Serial.print("Heading NW");
#endif

        if (curY < MIN_Y + 30) { // close to left wall 
            if (curX > MAX_X - 30) { // close to front wall
                nextWayPt();
            } else if (curX < MIN_X + 30) { // close to rear wall 
                if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            } else {
                if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            }
        } else if (curY > MAX_Y - 30) { // close to right wall
            if (curX > MAX_X - 30) { // close to front wall
                if (!obstacleLeft) {
                    mode = mEvadeLeft;
                } else {
                    scanForOpening();
                }
            } else if (curX < MIN_X + 30) { // close to rear wall
                if (!obstacleLeft) {
                    mode = mEvadeLeft;
                } else if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            } else {
                if (!obstacleLeft) {
                    mode = mEvadeLeft;
                } else {
                    scanForOpening();
                }
            }
        } else {
            if (!obstacleLeft) {
                mode = mEvadeLeft;
            } else if (!obstacleRight) {
                mode = mEvadeRight;
            } else {
                scanForOpening();
            }
        }
#if DEBUG_ROUTE
        Serial.println();
#endif
    } else if (theta < -1.57 && theta >= -3.14) { // pointing SW
#if DEBUG_ROUTE
            Serial.print("Heading SW");
#endif
        if (curY < MIN_Y + 30) { // close to left wall 
            if (curX > MAX_X - 30) { // close to front wall
                if (!obstacleLeft) {
                    mode = mEvadeLeft;
                } else {
                    scanForOpening();
                }
            } else if (curX < MIN_X + 30) { // close to rear wall 
                nextWayPt();
            } else {
                if (!obstacleLeft) {
                    mode = mEvadeLeft;
                } else {
                    scanForOpening();
                }
            }
        } else if (curY > MAX_Y - 30) { // close to right wall
            if (curX > MAX_X - 30) { // close to front wall
                if (!obstacleLeft) {
                    mode = mEvadeLeft;
                } else if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            } else if (curX < MIN_X + 30) { // close to rear wall
                if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            } else {
                if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            }
        } else {
            if (!obstacleLeft) {
                mode = mEvadeLeft;
            } else if (!obstacleRight) {
                mode = mEvadeRight;
            } else {
                scanForOpening();
            }
        }
#if DEBUG_ROUTE
        Serial.println();
#endif
    } else if (theta > 0 && theta < 1.57) { // pointing NE
#if DEBUG_ROUTE
            Serial.print("Heading NE");
#endif
        if (curY < MIN_Y + 30) { // close to left wall 
            if (curX > MAX_X - 30) { // close to front wall
                if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            } else if (curX < MIN_X + 30) { // close to rear wall 
                if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            } else {
                if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            }
        } else if (curY > MAX_Y - 30) { // close to right wall
            if (curX > MAX_X - 30) { // close to front wall
                nextWayPt();
            } else if (curX < MIN_X + 30) { // close to rear wall
                if (!obstacleLeft) {
                    mode = mEvadeLeft;
                } else if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            } else {
                if (!obstacleLeft) {
                    mode = mEvadeLeft;
                } else {
                    scanForOpening();
                }
            }
        } else {
            if (!obstacleLeft) {
                mode = mEvadeLeft;
            } else if (!obstacleRight) {
                mode = mEvadeRight;
            } else {
                scanForOpening();
            }
        }
#if DEBUG_ROUTE
        Serial.println();
#endif
    } else if (theta > 1.57 && theta <= 3.14) { // pointing SE
#if DEBUG_ROUTE
            Serial.print("Heading SE");
#endif
        if (curY < MIN_Y + 30) { // close to left wall 
            if (curX > MAX_X - 30) { // close to front wall
                if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            } else if (curX < MIN_X + 30) { // close to rear wall 
                if (!obstacleLeft) {
                    mode = mEvadeLeft;
                } else {
                    scanForOpening();
                }
            } else {
                if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            }
        } else if (curY > MAX_Y - 30) { // close to right wall
            if (curX > MAX_X - 30) { // close to front wall
                if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            } else if (curX < MIN_X + 30) { // close to rear wall
                nextWayPt();
            } else {
                if (!obstacleRight) {
                    mode = mEvadeRight;
                } else {
                    scanForOpening();
                }
            }
        } else {
            if (!obstacleRight) {
                mode = mEvadeRight;
            } else if (!obstacleLeft) {
                mode = mEvadeLeft;
            } else {
                scanForOpening();
            }
        }
#if DEBUG_ROUTE
        Serial.println();
#endif
    }
}

void lookForObstacle() {
    if (mode != mBackup && mode != mDropCan && mode != mGrabCan
            && mode != mDriveCan && !RobotBase.getTurning()
            && RobotBase.getX() < MAX_X - 15) {

        if (sonarDist < 40) {
            obstacleCenter = true;
        }
        if (RobotBase.irDistance(IRFL) < 25) {
            obstacleLeft = true;
        }
        if (RobotBase.irDistance(IRFR) < 25) {
            obstacleRight = true;
        }
        if (!obstacleCenter && !obstacleLeft && !obstacleRight) {
            if (turnAdjust > 0.01 || turnAdjust < -0.01) { // if there's no obstacle start reducing our turn adjust
                turnAdjust = turnAdjust / 2;
            }
            RobotBase.setTurnAdjust(turnAdjust);
            if (mode == mEvadeLeft || mode == mEvadeRight) {
                mode = nextMode;
            }
        } else {
            findRoute();
        }
    } // if mode...
}

void adjustRoute() {
    if (obstacleCenter && obstacleLeft && obstacleRight) {
        RobotBase.stop(false);
    } else if (obstacleCenter && obstacleLeft) {
        evalWayPoint(20, -0.79);
    } else if (obstacleCenter && obstacleRight) {
        evalWayPoint(20, 0.79);
    } else if (obstacleCenter) {
        if (mode == mEvadeLeft) {
            int diff = 40 - sonarDist;
            if (diff > 0) {
                turnAdjust = diff / turnFactor;
                RobotBase.setTurnAdjust(turnAdjust);
            }
        } else if (mode == mEvadeRight) {
            int diff = 40 - sonarDist;
            if (diff > 0) {
                turnAdjust = diff / turnFactor;
                RobotBase.setTurnAdjust(-turnAdjust);
            }
        } else if (obstacleLeft) {
            int diff = 40 - sonarDist;
            if (diff > 0) {
                turnAdjust = diff / turnFactor;
                RobotBase.setTurnAdjust(-turnAdjust);
            }
        } else if (obstacleRight) {
            int diff = 40 - sonarDist;
            if (diff > 0) {
                turnAdjust = diff / turnFactor;
                RobotBase.setTurnAdjust(turnAdjust);
            }
        }
    }

    obstacleCenter = false;
    obstacleLeft = false;
    obstacleRight = false;
}

TimedAction lookForObstacleAction = TimedAction(300, lookForObstacle);
