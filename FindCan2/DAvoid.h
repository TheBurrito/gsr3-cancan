bool obstacleCenter, obstacleLeft, obstacleRight;
double turnAdjust;
double turnFactor = 5;

double sonarAvoidDist = 40;
double sonarTurnFactor = 5;

double irAvoidDist = 15;
double irTurnFactor = 5;

typedef enum {
	aScan,
	aTurnAway,
	aDriveAround
} AvoidMode;

AvoidMode aMode = aScan;
bool turnLeft = true;
Point avoidTarget;

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
}

void lookForObstacle() {
        double x = RobotBase.getX();
        
    if (mode == mWander || mode == mDriveCan || (mode == mDriveGoal && x < MAX_X - 20)) {
            
        //return;
        
        //Variables to keep track of previous iteration values
        static int _s;
        
        double y = RobotBase.getY();
        
        int fl = RobotBase.irDistance(IRFL);
        int fr = RobotBase.irDistance(IRFR);
        int l = RobotBase.irDistance(IRL);
        int r = RobotBase.irDistance(IRR);
        
        bool front = sonarDist < sonarAvoidDist && sonarDist > 0;
        bool left = fl < irAvoidDist;
        bool right = fr < irAvoidDist;
        
        Point o;
        float fudge = 0;
        
        if (front && !takeOver) {
        	if (mode == mDriveCan) {
        		o.x = cans[targetCan].pos.x;
                o.y = cans[targetCan].pos.y;
                fudge = 5;
            } else {
		    	o.x = wayPts[wayPt].pos.x;
				o.y = wayPts[wayPt].pos.y;
			}
			
			float d = hypot(x - o.x, y - o.y);
			if (d < sonarDist + fudge) {
				front = false;
			}
		}
        
        static AvoidMode lastAMode;
        
        static double maxVel;
        
        bool newState = aMode != lastAMode;
        lastAMode = aMode;
        float turnAngle = 0;
        float turn = 0;
        
        switch (aMode) {
        case aScan:
        	if (front) {
        		aMode = aTurnAway;
    		}
    		break;
		
		case aTurnAway:
			if (newState) {
				if (left) {
					turnLeft = false;
				} else if (right) {
					turnLeft = true;
				} else if (l > r) {
					turnLeft = true;
				} else {
					turnLeft = false;
				}
				
				lcd.clear();
				lcd.setCursor(0,0);
				if (turnLeft) {
					lcd.print("left");
					turn = 0.5;
				} else {
					lcd.print("right");
					turn = -0.5;
				}
					
				RobotBase.setVelocityAndTurn(0.0, turn, true);
			} else {
				if (!front) {
					o = getSensorPoint(_s, sonarPose);
					o.x -= x;
					o.y -= y;
					
					turnAngle = 2 * asin(6 / _s);
					if (!turnLeft) {
						turnAngle *= -1;
					}
					o = rotate(o, turnAngle);
					
					o.x += x;
					o.y += y;
					
					avoidTarget = o;
					aMode = aDriveAround;
				}
			}
			break;
		
		case aDriveAround:
			if (newState) {
				lcd.clear();
				lcd.setCursor(0,0);
				lcd.print("Avoid ");
				lcd.print(avoidTarget.x);
				lcd.print(",");
				lcd.print(avoidTarget.y);
				
				takeOver = true;
				maxVel = RobotBase.getMaxVel();
				RobotBase.setMaxVel(20);
				RobotBase.turnToAndDrive(avoidTarget.x, avoidTarget.y, true);
			} else {
				if (RobotBase.navDone()) {
					
					lcd.clear();
					lcd.setCursor(0,0);
					lcd.print("r");
					
					RobotBase.setMaxVel(maxVel);
					RobotBase.turnToAndDrive(wayPts[wayPt].pos.x, wayPts[wayPt].pos.y, true);
					takeOver = false;
					aMode = aScan;
					lcd.clear();
					lcd.setCursor(0,0);
					lcd.print("Resume");
				}
			}
			break;
		}
		
		_s = sonarDist;
        
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
