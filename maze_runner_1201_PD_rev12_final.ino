//Threshold IR readout for line detection
//Input independent values for each variable
const int threshold1 = 200; 
const int threshold2 = 200;
const int threshold3 = 200;
const int threshold4 = 200;
const int threshold5 = 200;
const int threshold6 = 200;
const int desired3 = 670;
const int desired4 = 730;


int offMaze3 = 850;
int offMaze4 = 850;
//int wallThreshold;

//Digital responses to line sensors - use boolean
boolean WhiteWall;
boolean digiCP;
boolean rev;
//boolean air;

//extra variables
boolean switchBias;
boolean cpCheck;    //testing
int tempCounter;    //testing
boolean Victory;
int Turn_Counter;
int SecLeft_Counter;
int SecRight_Counter;
boolean secStop;
boolean secCheck;
boolean uturnCheck;
boolean tempCP;
int cpCounter;    //testing
int masterCP;
boolean FreeRight;
boolean FreeLeft;

boolean flag1;

//Wall Detection Variables
long pulseTime;
long wallDist; 
const int stopDist = 6;
const int readDist = 20;


//PD Variables
int RightError;
int PrevRightError;
int LeftError;
int PrevLeftError;
int lmSpeed;
int rmSpeed;



const float Kp = 0.5;
const float Kd = 0.5;

int testLED3;
int testLED4;


//IR Pins - from left to right when rover is right side up
//Assign analog input pins to each variable 
#define IR1 A0
#define IR2 A1 
#define IR3 A2 
#define IR4 A3 
#define IR5 A4
#define IR6 A5

//Objective Pins - Checkpoint and Wall
//Assign analog input pins to each variable 
#define LED1 2
#define LED2 3
#define checkpoint 4
#define wallRecv 11
#define Trigger 12
#define Echo 13


//Motor Pins - PWM
//Assign digital pins to each variable
#define LeftFwd 5  //Left Forward ->  -ve term 
#define LeftRev 6  //Left Reverse ->  +ve term
#define RightFwd 10  //Right Forward  ->  +ve
#define RightRev 9  //Right Reverse ->  -ve

//Speed Values
#define HSpeed 200    //High Speed PWM
#define TSpeed 200    //Medium Speed PWM
#define ISpeed 175     //Low Speed PWM
#define Zero 0        //Zero PWM

//Variables to store analog readouts from line sensors
int left1;
int left2;
int mid3; //centre pin 1
int mid4; //centre pin 2  
int right5;
int right6;

//Delay constants
int left_Time = 500;
int right_Time = 400;
#define leftShort_Time 200
#define rightShort_Time 200
#define uturn_Time 1500
#define miniRev_Time 200
#define cross_Time 650
#define Inch_Time 300
#define revInch_Time 500
#define stop_Time 3000



//int init_brake;
//int reverse;
//int intersection;


void setup() {
  
  pinMode(IR1,INPUT);
  pinMode(IR2,INPUT);
  pinMode(IR5,INPUT);
  pinMode(IR6,INPUT);

  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(checkpoint,INPUT);
  pinMode(Trigger,OUTPUT);
  pinMode(Echo,INPUT);
  pinMode(wallRecv,INPUT);
  

  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(Trigger,LOW);
   
  Serial.begin(9600);

  //Initial Conditions

  cpCounter = 0;
  switchBias = false;
  cpCheck = false;
  WhiteWall = false;
  Victory = false;
  
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  
  secCheck = false;
  secStop = false;
  tempCP = true;
  uturnCheck = false;
  FreeRight = false;
  FreeLeft = false;
  wallDist = 170;

  flag1 = false;
  
  PrevRightError = 50;
  PrevLeftError = 50;

  Turn_Counter = 0;
  SecLeft_Counter = 0;
  SecRight_Counter = 0;

  ResetMotor();

  
}

void loop() {

  top:
  
  // inputVal();     //Serial print values of each IR Sensor
     PrintVal();
  
     
  checkEStop();   //Check if maze is solved

  if(Victory)
  {
    goto top;
  }

    if(mid3 >= offMaze3 && mid4 >= offMaze4)   //Check if rover is off the maze
    {
      ResetMotor();
      goto top;
    }
  

   
   RightError = mid4 - desired4;
   lmSpeed = HSpeed + Kp * RightError + Kd * (RightError - PrevRightError); 
   PrevRightError = RightError;
   if(lmSpeed < 0)
   {
      lmSpeed = 0; 
   }
   else if(lmSpeed > 255)
   {
      lmSpeed = 255;
   }
   Drive("leftPD",0,lmSpeed);
   

   LeftError = mid3 - desired4;
   rmSpeed = HSpeed + Kp * LeftError + Kd * (LeftError - PrevLeftError); 
   PrevLeftError = LeftError; 
   if(rmSpeed < 0)
   {
      rmSpeed = 0;
   }
   else if(rmSpeed > 255)
   {
      rmSpeed = 255;
   }
   Drive("rightPD",0,rmSpeed);


  
//INTERSECTION

    inputVal();
	
	//OBSTACLE DETECTION
    wallDetection();  
	  cpDetection(digitalRead(checkpoint));

    if(left1>=threshold1 && left2>=threshold2 && right5>=threshold5 && right6>=threshold6)    //Coming into a T or Cross Intersection
    {       
		      FreeRight = false;
          FreeLeft = false;
		      
		      Drive("Inch",cross_Time,ISpeed);
          inputVal();

          if(mid3 >= threshold3 || mid4 >= threshold4)    //Check for Cross 
          {
//            goto top;
          }
          else      //T Intersection
          {
              Turn_Counter = Turn_Counter + 1;
                
              if(!switchBias)      
              {
                  if(Turn_Counter == 2)
                  {
                    SecondaryRightCheck();
                  }
                  else
                  {
                    Drive("revInch",revInch_Time,ISpeed);
                    TurnLeft();
                    Turn_Counter = 0;
                    SecLeft_Counter = 0;
                    SecRight_Counter = 0;
                  }
              }
              else
              {
                  if(Turn_Counter == 2)
                  {
                    SecondaryLeftCheck();       
                  }
                  else
                  {
                    Drive("revInch",revInch_Time,ISpeed);
                    TurnRight();
                    Turn_Counter = 0;
                    SecLeft_Counter = 0;
                    SecRight_Counter = 0;
                  }
              }         
          }  

          uturnCheck = false;
    }
    else if((left1>=threshold1 && left2>=threshold2) || (right5>=threshold5 && right6>=threshold6))   //Free Turn or Coming into T on a branch
    {       
          
          if(!switchBias)      
          {
            leftBias();                       
          }
          else
          {
            rightBias();         
          }    

          uturnCheck = false;  
    }
    else if(mid3 < threshold3 && mid4 < threshold4)     //Dead end
    {
      FreeRight = false;
      FreeLeft = false;
      
      uturnCheck = true;
	    tempCP = true;
	    TurnRight();
    }

    if(left1<threshold1 && left2<threshold2 && right5<threshold5 && right6>=threshold6)   //For Bot Right Going Up w/ Left Bias
    {
      Drive("Inch",Inch_Time+200,ISpeed);   //Increase Inch_Time for calibration  
    }


//Post Intersection

	if(uturnCheck && cpCounter == 2)
	{
		uturnCheck = false;
		tempCP = true;
		masterCP = masterCP + 1;
		cpCounter = 0;
		cpCheck = true;
	}
	
	if(masterCP == 1)
	{
		digitalWrite(LED1,HIGH);
	}
	else if(masterCP == 2)
	{
//		digitalWrite(LED2,HIGH);    //for testing switch bias turn off this test
      digitalWrite(LED1,LOW);
	}
 
   
    // PrintVal();

}


void HardStop()
{

  analogWrite(LeftFwd,Zero);
  analogWrite(RightFwd,Zero);
  analogWrite(LeftRev,Zero);
  analogWrite(RightRev,Zero);
  delay(3000);   //To give motor enough time to switch direction

}


void leftBias()
{

    left_Time = 450;
    right_Time = 400;

    if (left1>=threshold1 && left2>=threshold2 && right5<threshold5 && right6<threshold6)   
    //Primary -> Check for only left-turn -> GO LEFT
    {
          Drive("Inch",Inch_Time+100,ISpeed);
          TurnLeft();

    }
    
    else if ((left1<threshold1 && left2<threshold2 && right5>=threshold5 && right6>=threshold6)&&(!secStop))    
    //Secondary -> Check for only right-turn -> GO RIGHT
    {
        
		    Drive("Inch",Inch_Time+100,ISpeed);  
        inputVal();
        
        if(mid3 >= threshold3 || mid4 >= threshold4)
        {
          Turn_Counter = Turn_Counter + 1;
          SecRight_Counter = SecRight_Counter + 1;
          FreeRight = false;
        }
        else
        {
          FreeRight = true;
        }

//        if(secCheck)
//        {
//          SecRight_Counter = SecRight_Counter + 1;
//        }

//        SecRight_Counter = SecRight_Counter + 1;

        

        if(Turn_Counter == 0)
        {
          TurnRight();
        }
        else if(Turn_Counter == 1)
        {
            secCheck = true;
			      TurnRight();	
			      cpCheck = false;      //Clear previous status of checkpoint

            if(FreeRight)
            {
                digitalWrite(LED2,HIGH);
                switchBias = true;
                secStop = false;
                Turn_Counter = 0;
                SecRight_Counter = 0; 
                SecLeft_Counter = 0;
            }
           
        }
        else if(Turn_Counter == 2)
        {
            SecondaryRightCheck();
        }
        else if(Turn_Counter > 2)
        {
          Turn_Counter = 0;
          SecRight_Counter = 0; 
          SecLeft_Counter = 0; 
		      TurnRight();
        }
        
    }
  
}

void SecondaryRightCheck()
{

            if(SecRight_Counter == 0)
            {
              Turn_Counter = 0;
              SecRight_Counter = 0;
              TurnRight();
            }
            else if(SecRight_Counter == 1)
            {
              if(cpCheck)   //the line had an intersection
              {
                secStop = true;
                if(!switchBias)   //execute opposite of left bias for one instance
                {
                  TurnRight;
                }
                else        //execute opposite of right bias for one instance
                { 
                  TurnLeft;
                }
              }
              else
              {
                digitalWrite(LED2,HIGH);
                switchBias = true;
                secStop = false;
                TurnRight();
              }
        
              Turn_Counter = 0;
              SecLeft_Counter = 0;
              SecRight_Counter = 0;
            }
      
}


void rightBias()
{

    left_Time = 700;
    right_Time = 500;
    
    if (left1<threshold1 && left2<threshold2 && right5>=threshold5 && right6>=threshold6)    
    //Primary -> Check for only right-turn -> GO RIGHT
    {
        Drive("Inch",Inch_Time+200,ISpeed);  
        TurnLeft();
    }

    else if((left1>=threshold1 && left2>=threshold2 && right5<threshold5 && right6<threshold6)&&(!secStop))   
    //Secondary -> Check for only left-turn -> GO LEFT
    {

		
    		Drive("Inch",Inch_Time+200,ISpeed);
    		inputVal();

        if(mid3 >= threshold3 || mid4 >= threshold4)
        {
          Turn_Counter = Turn_Counter + 1;
          SecLeft_Counter = SecLeft_Counter + 1;
          FreeLeft = false;
        }
        else
        {
          FreeLeft = true;
        }

//        if(secCheck)
//        {
//          SecLeft_Counter = SecLeft_Counter + 1;
//        }

//        SecLeft_Counter = SecLeft_Counter + 1;
		
		    if(Turn_Counter == 1)
        {
            secCheck = true;
			      TurnLeft();	
            cpCheck = false;

            if(FreeLeft)
            {
              digitalWrite(LED2,LOW);
              switchBias = false;
              secStop = false;
              Turn_Counter = 0;
              SecRight_Counter = 0; 
              SecLeft_Counter = 0;
            }
        }
        else if(Turn_Counter == 2)
        {
            SecondaryLeftCheck();
        }
        else if(Turn_Counter > 2)
        {
          Turn_Counter = 0;
          SecRight_Counter = 0; 
          SecLeft_Counter = 0;
		      TurnLeft();		  
        }
		
    }
  
}


void SecondaryLeftCheck()
{
            if(SecLeft_Counter == 0)
            {
              Turn_Counter = 0;
              SecLeft_Counter = 0;
              TurnLeft();
            }
            else if(SecLeft_Counter == 1)
            {
              if(cpCheck)   //the line had an intersection
              {
                  secStop = true;
                  if(!switchBias)   //execute opposite of left bias for one instance
                  {
                    TurnRight;
                  }
                  else        //execute oppostie of right bias for one instance
                  { 
                    TurnLeft;
                  }
              }
              else
              {
                digitalWrite(LED2,LOW);
                switchBias = false;
                secStop = false;
                TurnLeft;
              }
              
              Turn_Counter = 0;
              SecLeft_Counter = 0;
              SecRight_Counter = 0;
            }
}

void cpDetection(boolean digiCP)
{
  if (!digiCP && tempCP)
  {
     cpCounter = cpCounter + 1;
	   tempCP = false;
  } 
}


void wallDetection()
{

    
//    digitalWrite(Trigger,LOW);  
//    delayMicroseconds(2);     //to ensure clean signal
//    digitalWrite(Trigger,HIGH);
//    delayMicroseconds(10);
//    digitalWrite(Trigger,LOW);
//
//    pulseTime = pulseIn(Echo,HIGH);
// 
//    wallDist = (pulseTime/2)/29;    //for distance in cm\

    ultrasonic();
    PrintVal();

    

    if(wallDist == 0)
    {
      
    }
    else if (wallDist <= stopDist)   //Rover is too close to wall
    {
	   
       
         Drive("rev",0,ISpeed);
         while(wallDist < readDist)
         {
            ultrasonic();
            flag1 = true;
            PrintVal();
         }
         ResetMotor();

         if(!digitalRead(wallRecv))    //White Wall
         {
            WhiteWall = true;
         }

       Drive("Inch",Inch_Time,ISpeed);
       TurnRight();
//       Drive("HardStop",stop_Time,HSpeed);
      
    }

}

void ultrasonic()
{
    digitalWrite(Trigger,LOW);  
    delayMicroseconds(2);     //to ensure clean signal
    digitalWrite(Trigger,HIGH);
    delayMicroseconds(10);
    digitalWrite(Trigger,LOW);

    pulseTime = pulseIn(Echo,HIGH);
 
    wallDist = (pulseTime/2)/29;    //for distance in cm\
  
}


void checkEStop()
{
  if(WhiteWall && masterCP >= 2)
  {
    Victory = true;
    HardStop();
  }
  else
  {
    WhiteWall = false;
  }
}


void ResetMotor()
{
  analogWrite(LeftFwd,Zero);
  analogWrite(RightFwd,Zero);
  analogWrite(LeftRev,Zero);
  analogWrite(RightRev,Zero);
//  delay(200);
}


void TurnLeft()
{
  Drive("left",left_Time,TSpeed);
  
  do 
  {
    Drive("LeftCont",0,HSpeed);
    inputVal();
  }while(!(mid4>threshold4 && right5<threshold5));
    Drive("Inch",Inch_Time,ISpeed);
}


void TurnRight()
{
  Drive("right",right_Time,TSpeed);

  do 
  {  
    Drive("RightCont",0,HSpeed);
    inputVal();
  }while(!(left2<threshold2 && mid3>threshold3));
    Drive("Inch",Inch_Time,ISpeed);
  
}


void Drive(char Direction[20], int Duration, int DriveSpeed)
{

  if (Direction == "fwd")
  {
      analogWrite(LeftFwd,DriveSpeed);
      analogWrite(RightFwd,DriveSpeed);
      analogWrite(LeftRev,Zero);
      analogWrite(RightRev,Zero);
      return;
  }
  
  if (Direction == "rev")
  {
      analogWrite(LeftFwd,Zero);
      analogWrite(RightFwd,Zero);
      analogWrite(LeftRev,DriveSpeed);
      analogWrite(RightRev,DriveSpeed);
      return;
  }
  

  if (Direction == "leftPD")
  {
      analogWrite(LeftFwd,DriveSpeed);
//      analogWrite(RightFwd,Zero);
      analogWrite(LeftRev,Zero);
      analogWrite(RightRev,Zero);
      return;
  }

  if (Direction == "rightPD")
  {
//      analogWrite(LeftFwd,Zero);
      analogWrite(RightFwd,DriveSpeed);
      analogWrite(LeftRev,Zero);
      analogWrite(RightRev,Zero);
      return;
  }

  else if(Direction == "RightCont")
  {
      analogWrite(LeftFwd,DriveSpeed);
      analogWrite(RightFwd,Zero);
      analogWrite(LeftRev,Zero);
      analogWrite(RightRev,DriveSpeed);
      return;
  }

  else if(Direction == "LeftCont")
  {
      analogWrite(LeftFwd,Zero);
      analogWrite(RightFwd,DriveSpeed);
      analogWrite(LeftRev,DriveSpeed);
      analogWrite(RightRev,Zero);
      return;
  }

  else if(Direction == "left")
  {
      analogWrite(LeftFwd,Zero);
      analogWrite(RightFwd,DriveSpeed);
      analogWrite(LeftRev,DriveSpeed);
      analogWrite(RightRev,Zero);
      delay(Duration);
      return;
  }

  else if(Direction == "right")
  {
      analogWrite(LeftFwd,DriveSpeed);
      analogWrite(RightFwd,Zero);
      analogWrite(LeftRev,Zero);
      analogWrite(RightRev,DriveSpeed);
      delay(Duration);
      return;
  }
  
  else if(Direction == "Inch")
  {
      analogWrite(LeftFwd,DriveSpeed);
      analogWrite(RightFwd,DriveSpeed);
      analogWrite(LeftRev,Zero);
      analogWrite(RightRev,Zero);
      delay(Duration);
  }

  else if(Direction == "revInch")
  {
      analogWrite(LeftFwd,Zero);
      analogWrite(RightFwd,Zero);
      analogWrite(LeftRev,DriveSpeed);
      analogWrite(RightRev,DriveSpeed);
      delay(Duration);
  }

  else if(Direction == "HardStop")
  {
      analogWrite(LeftFwd,Zero);
      analogWrite(RightFwd,Zero);
      analogWrite(LeftRev,Zero);
      analogWrite(RightRev,Zero);
      delay(Duration);
  }

  else
  {
    Drive("HardStop",stop_Time,HSpeed);
  }

  ResetMotor();
  return;
  
}

void PrintVal()   
{
  
  inputVal();
  
  Serial.print(left1);
  Serial.print(" ");
  Serial.print(left2);
  Serial.print(" ");
  Serial.print(mid3);
  Serial.print(" ");
  Serial.print(mid4);
  Serial.print(" ");
  Serial.print(right5);
  Serial.print(" ");
  Serial.print(right6);
  Serial.print("\t");  
  Serial.print(digitalRead(checkpoint));
  Serial.print("\t");
  Serial.print(wallDist);
  Serial.print(" ");
  Serial.print(digitalRead(wallRecv));
  Serial.print("\t");
  Serial.print(switchBias);
  Serial.print("\t");
  Serial.println(flag1);     //TESTING

}

void inputVal()
{
  left1 = analogRead(IR1);
  left2 = analogRead(IR2);
  mid3 = analogRead(IR3);
  mid4 = analogRead(IR4);
  right5 = analogRead(IR5);
  right6 = analogRead(IR6);
}

