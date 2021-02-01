  #define NUM_MOTOR 3 //모터의 수
  
  //호밍 상태 정의 상수
  #define MODE_NOTHOME 0 //모드: 호밍필요
  #define MODE_HOME 1 //모드: 호밍완료
  #define MODE_STOP 0 //모드: 정지
  #define MODE_MOVE 1 //모드: 동작

  //포트 초기값 정의
  #define X_STEP_HIGH        PORTD |=  0b00000100;         //핀 2 HIGH로 지정
  #define X_STEP_LOW         PORTD &= ~0b00000100;         //핀 2 LOW로 지정
  #define Y_STEP_HIGH        PORTD |=  0b00001000;         //핀 3 HIGH로 지정
  #define Y_STEP_LOW         PORTD &= ~0b00001000;         //핀 3 LOW로 지정
  #define Z_STEP_HIGH        PORTD |=  0b00010000;         //핀 4 HIGH로 지정
  #define Z_STEP_LOW         PORTD &= ~0b00010000;         //핀 4 LOW로 지정

  //타이머 설정
  #define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
  #define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

  //홈 위치 지정
  #define POS_HOME_X 1167
  #define POS_HOME_X 950
  #define POS_HOME_X 1130

  //핀 번호 모음
  #define dirPin_X 2
  #define dirPin_Y 3
  #define dirPin_Z 4
  #define stepPin_X 5
  #define stepPin_Y 6
  #define stepPin_Z 7
  #define sensorPin_X 9
  #define sensorPin_Y 10
  #define sensorPin_Z 11

  //좌표변수
  int posX = 0;
  int posY = 0;
  int posZ = 46;

  ///초기좌표
  int initX = 0;
  int initY = 0;
  int initZ = 46;

  ///좌표 이전값 저장
  int prevX;
  int prevY;
  int prevZ;

  ///좌표 범위
  int maxX = 5;
  int minX = -25;
  int maxY = 17;
  int minY = -17;
  int maxZ = 58;
  int minZ = 34;

  //스텝모터를 표현하는 구조체(*모든 정보를 통합)
  struct stepperInfo {
    //고정된 상수값
    float acceleration; //가속도값(클수록 증가)
    volatile unsigned long minStepInterval; //최대속도 결정(작을수록 빠름)
    bool homing; //현재호밍상태
    bool moving; //현재동작상태
    void(*dirFunc)(int); //모터의 방향함수(아래에 정의됨)
    void(*stepFunc)(); //모터의 스텝펄스함수(아래에 정의됨)
    void(*homeFunc)(int); //모터의 호밍함수(아래에 정의됨)

    //계산되는 상수값
    unsigned int v0; //초기속도delay값(클수록 감소)
    long stepPos; //  현재 스텝위치

    //이동에 따라 변하는 변수값
    volatile int dir;                        //현재 동작방향
    volatile unsigned int totalSteps;        //현재 요청된 이동스텝수
    volatile bool movementDone = false;      //동작완료여부
    volatile unsigned int rampUpStepCount;   //최대 속도 or 중간 지점까지의 스텝수 계산값 초기화
    volatile unsigned long estStepsToSpeed;  //최대 속도까지의 스텝
    volatile unsigned long estTimeForMove;   //최대 속도까지의 시간
    volatile unsigned long rampUpStepTime;
    volatile float speedScale;               //속도 동기화를 위해 속도에 곱해지는 값

    // per iteration variables (potentially changed every interrupt)
    volatile unsigned int n;                 //가속도 관련 카운터
    volatile float d;                        //가속도 관련 간격 초기화
    volatile unsigned long di;               //n과 d의 계산값 초기화
    volatile unsigned int stepCount;         //현재 스텝수
  }

  //Inverse Kinematics 판별 후 좌표 입력 함수
  bool isDataexist();
  bool coordrange();
  void coordprint();
  void homefunction();


  //좌표 입력여부 판별
  bool xreceived = false;
  bool yreceived = false;
  bool zreceived = false;

  //입력값을 범위에 맞추어 변환하는 함수
  float mapping(float x, float minFrom, float maxFrom, float minTo, float maxTo);
  //라디안-각도, 각도-라디안 변환 함수
  float deg2rad(float a);
  float rad2deg(float a);
  //계산된 각도를 스텝모터의 스텝 수로 계산하는 함수
  int angle2step(float deg);
  //Inverse Kinematics를 계산하는 함수
  bool getDegree_byFunction(float x, float y, float z, float* deg1, float* deg2, float* deg3);

  //초기값 지정함수
  void stepX() {
    X_STEP_HIGH
    X_STEP_LOW
  }

  void dirX(int dir) {
    digitalWrite(dirPin_X, dir); //방향표기 중요
  }

  void stepY() {
    Y_STEP_HIGH
    Y_STEP_LOW
  }

  void dirY(int dir){
    digitalWrite(dirPin_Y, dir); //방향표기 중요
  }

  void stepZ() {
    Z_STEP_HIGH
    Z_STEP_LOW
  }

  void dirZ(int dir){
    digitalWrite(dirPin_Z, dir); //방향표기 중요
  }

  //스텝모터 구조체 초기화용 함수
  void resetStepperInfo( stepperInfo& si ) {
  si.n = 0; //가속도 관련 카운터 초기화
  si.d = 0; //가속도 관련 간격 초기화
  si.di = 0; //n과 d의 계산값 초기화
  si.stepCount = 0; //현재 스텝수 초기화
  si.rampUpStepCount = 0; //최대 속도 or 중간 지점까지의 스텝수 계산값 초기화
  si.rampUpStepTime = 0; //위의 값을 스텝이 아닌 시간으로 계산(*현재 미사용)하여 초기화
  si.totalSteps = 0; //현재 요청된 이동스텝수 초기화
  si.stepPos = 0; //현재 스텝위치 초기화
  si.movementDone = false; //동작완료여부 초기화
  }

  //모터 3개에 대한 구조체
  volatile stepperInfo steppers[NUM_MOTOR];

  void setup() {
    pinMode(stepPin_X, OUTPUT);
    pinMode(dirPin_X, OUTPUT);
    pinMode(sensorPin_X, INPUT);

    pinMode(stepPin_Y, OUTPUT);
    pinMode(dirPin_Y, OUTPUT);
    pinMode(sensorPin_Y, INPUT);

    pinMode(stepPin_Z, OUTPUT);
    pinMode(dirPin_Z, OUTPUT);
    pinMode(sensorPin_Z, INPUT);

    digitalWrite(sensorPin_X, LOW);
    digitalWrite(sensorPin_Y, LOW);
    digitalWrite(sensorPin_Z, LOW);

    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    OCR1A = 1000;                             // compare value
    TCCR1B |= (1 << WGM12);                   // CTC mode
    TCCR1B |= ((1 << CS11) | (1 << CS10));    // 64 prescaler
    interrupts();

    steppers[0].dirFunc = dirX;
    steppers[0].stepFunc = stepX;
    steppers[0].acceleration = 1000;
    steppers[0].minStepInterval = 50;

    steppers[1].dirFunc = dirY;
    steppers[1].stepFunc = stepY;
    steppers[1].acceleration = 1000;
    steppers[1].minStepInterval = 50;

    steppers[2].dirFunc = dirZ;
    steppers[2].stepFunc = stepZ;
    steppers[2].acceleration = 1000;
    steppers[2].minStepInterval = 50;
  }

  void resetStepper(volatile stepperInfo& si) {
    si.v0 = si.acceleration;
    si.d = si.v0;
    si.di = si.d;
    si.stepCount = 0;
    si.n = 0;
    si.rampUpStepCount = 0;
    si.movementDone = false;
    si.speedScale = 1;

    si.moving = false;
    si.homing = false;

    float a = si.minStepInterval / (float)si.v0;
    a *= 0.676;

    float m = ((a*a - 1) / (-2 * a));
    float n = m * m;

    si.estStepsToSpeed = n;
  }

  volatile byte remainingSteppersFlag = 0;

  float getDurationOfAcceleration(volatile stepperInfo& s, unsigned int numSteps) {
    float d = s.v0;
    float totalDuration = 0;
    for (unsigned int n = 1; n < numSteps; n++) {
      d = d - (2 * d) / (4 * n + 1);
      totalDuration += d;
    }
    return totalDuration;
  }

  void prepareMovement(int whichMotor, long steps) {
    volatile stepperInfo& si = steppers[whichMotor];
    si.dirFunc( steps < 0 ? HIGH : LOW );
    si.dir = steps > 0 ? 1 : -1;
    si.totalSteps = abs(steps);
    resetStepper(si);
    
    remainingSteppersFlag |= (1 << whichMotor);

    unsigned long stepsAbs = abs(steps);

    if ( (2 * si.estStepsToSpeed) < stepsAbs ) {
      // there will be a period of time at full speed
      unsigned long stepsAtFullSpeed = stepsAbs - 2 * si.estStepsToSpeed;
      float accelDecelTime = getDurationOfAcceleration(si, si.estStepsToSpeed);
      si.estTimeForMove = 2 * accelDecelTime + stepsAtFullSpeed * si.minStepInterval;
    }
    else {
      // will not reach full speed before needing to slow down again
      float accelDecelTime = getDurationOfAcceleration( si, stepsAbs / 2 );
      si.estTimeForMove = 2 * accelDecelTime;
    }
  }

  volatile byte nextStepperFlag = 0;

  void setNextInterruptInterval() {

    bool movementComplete = true;

    unsigned long mind = 999999;
    for (int i = 0; i < NUM_MOTOR; i++) {
      if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di < mind ) {
        mind = steppers[i].di;
      }
    }

    nextStepperFlag = 0;
    for (int i = 0; i < NUM_MOTOR; i++) {
      if ( ! steppers[i].movementDone )
        movementComplete = false;
      if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di == mind )
        nextStepperFlag |= (1 << i);
    }

    if ( remainingSteppersFlag == 0 ) {
      TIMER1_INTERRUPTS_OFF
      OCR1A = 65500;
    }

    OCR1A = mind;
  }

  ISR(TIMER1_COMPA_vect)
  {
    unsigned int tmpCtr = OCR1A;

    OCR1A = 65500;

    for (int i = 0; i < NUM_STEPPERS; i++) {

      if ( ! ((1 << i) & remainingSteppersFlag) )
        continue;

      if ( ! (nextStepperFlag & (1 << i)) ) {
        steppers[i].di -= tmpCtr;
        continue;
      }

      volatile stepperInfo& s = steppers[i];

      if ( s.stepCount < s.totalSteps ) {
        s.stepFunc();
        s.stepCount++;
        s.stepPosition += s.dir;
        if ( s.stepCount >= s.totalSteps ) {
          s.movementDone = true;
          remainingSteppersFlag &= ~(1 << i);
        }
      }

      if ( s.rampUpStepCount == 0 ) {
        s.n++;
        s.d = s.d - (2 * s.d) / (4 * s.n + 1);
        if ( s.d <= s.minStepInterval ) {
          s.d = s.minStepInterval;
          s.rampUpStepCount = s.stepCount;
        }
        if ( s.stepCount >= s.totalSteps / 2 ) {
          s.rampUpStepCount = s.stepCount;
        }
        s.rampUpStepTime += s.d;
      }
      else if ( s.stepCount >= s.totalSteps - s.rampUpStepCount ) {
        s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);
        s.n--;
      }

      s.di = s.d * s.speedScale; // integer
    }

    setNextInterruptInterval();

    TCNT1  = 0;
  }

  void runAndWait() {
    adjustSpeedScales();
    setNextInterruptInterval();
    TIMER1_INTERRUPTS_ON
    while ( remainingSteppersFlag );
    remainingSteppersFlag = 0;
    nextStepperFlag = 0;
  }

  void adjustSpeedScales() {
    float maxTime = 0;
    
    for (int i = 0; i < NUM_MOTOR; i++) {
      if ( ! ((1 << i) & remainingSteppersFlag) )
        continue;
      if ( steppers[i].estTimeForMove > maxTime )
        maxTime = steppers[i].estTimeForMove;
    }

    if ( maxTime != 0 ) {
      for (int i = 0; i < NUM_MOTOR; i++) {
        if ( ! ( (1 << i) & remainingSteppersFlag) )
          continue;
        steppers[i].speedScale = maxTime / steppers[i].estTimeForMove;
      }
    }
  }

  void loop() {

    while(!Serial.available())
    {
      Serial.println("Enter h to start homing.");
      delay(3000);
    }

    //동작을 시작
    while(Serial.available())
    { 
      while(!homeend)
      {
        if (Serial.read() == 'h')
        {
          homestart = true;
          Serial.println("Homing Started");
          for (int i = 0; i < NUM_MOTOR; i++)
          {
            if (i < 2)
            {
              homefunction(i, true);
            }
            else
            {
              homefunction(i, false);
            }
          }
          Serial.println("Homing Complete");
          homeend = true;
          //동작 출발
        }
        Serial.flush();
        delay(500);
      }

    for (int i = 0; i < NUM_STEPPERS; i++) {
      prepareMovement( i, 800 );
      runAndWait();
    }

    prepareMovement( 0, 8000 );
    prepareMovement( 1,  800 );
    prepareMovement( 2, 2400 );

    runAndWait();
    
    

      Serial.println("Enter c to input coordinates.");
      while(!state.pre)
      {
        if(Serial.read() == 'c')
        {
          state.pre = MODE_MOVE;
          Serial.flush();
        }
      }

      //좌표입력 시작
      while(state.pre && !state.cur)
      {
        //3차원 좌표 시리얼 모니터 상에서 입력
        Serial.println("Enter Coordinates x, y, z.");
        while(!xreceived)
        {
          if(!(posX || posX == 0))
          {
            posX = Serial.parseInt();
            Serial.flush();
          }
          else
          {
            Serial.println("xreceived");
            Serial.println(posX);
            xreceived = true;
          }
        }
        while(!yreceived)
        {
          if(!(posY || posY == 0))
          {
            posY = Serial.parseInt();
            Serial.flush();
          }
          else
          {
            Serial.println("yreceived");
            Serial.println(posY);
            yreceived = true;
          }
        }
        while(!zreceived)
        {
          if(!(posZ || posZ == 0))
          {
            posZ = Serial.parseInt();
            Serial.flush();
          }
          else
          {
            Serial.println("zreceived");
            Serial.println(posZ);
            zreceived = true;
          }
        }
        if(coordrange(posX, posY, posZ))
        {
          Serial.println("Coordidnates in range.");
          state.cur = MODE_MOVE;
        }
        //좌표가 범위 밖인 경우 처음부터 재입력
        else
        {
          Serial.println("Coordinates out of range!");
          posX = initX;
          posY = initY;
          posZ = initZ;
          xreceived = false;
          yreceived = false;
          zreceived = false;
        }
      }

      delay(100);

      while(state.cur)
      { 
        //읽어들인 위치값을 바탕으로 역기구학 계산
        bool isDataExist = false;       //유효한 위치인지를 판별하기 위한 변수 생성
        float deg[NUM_MOTOR];           //계산된 각 모터의 각도를 저장할 변수 생성
        //posX, posY, posZ를 통해 deg0, deg1, deg2를 계산. 유효한 좌표의 경우 isDataExist에 true저장
        isDataExist = getDegree_byFunction(posX, posY, posZ, &deg[0], &deg[1], &deg[2]);
        //모터의 부착 위치에 맞게 각도 수정
        if(isDataExist)
        {
          //좌표출력
          Serial.println("Input coordinates are:");
          Serial.print("posX:");
          Serial.println(posX);
          Serial.print("posY:");
          Serial.println(posY);
          Serial.print("posZ:");
          Serial.println(posZ);
          //deg출력
          Serial.print("deg0 is:");
          Serial.println(deg[0]);
          Serial.print("deg1 is:");
          Serial.println(deg[1]);
          Serial.print("deg2 is:");
          Serial.println(deg[2]);
        }
        else
        {
          Serial.println("Out of box.");
        }
        if(isDataExist)
        {

          //초기속도
          float vel[3] = {1e+3, 1e+3, 1e+3};
          float del[3] = {1e+3, 1e+3, 1e+3};
          float dim = 0.25;
          int multiplier = 0.01;
          int threshold  = 1e+3 * multiplier;
          float tconst = 1.37;

          deg[0] = -deg[0];
          deg[1] = -deg[1];

          //계산된 각도에 맞추어 해당 위치로 가기 위한 스텝 수와 방향을 설정
          for(int i = 0; i<NUM_MOTOR; i++)
          {
            motor[i].steps.cur = angle2step(deg[i]);    //각도를 스텝 수로 변환          
            //현위치와 목적 위치의 차이가 10스텝 이하라면 무시
            motor[i].steps.err = (abs(motor[i].steps.cur - motor[i].steps.pre)<1)?0:motor[i].steps.cur - motor[i].steps.pre;
            motor[i].steps.pre = motor[i].steps.cur;    //이전 위치를 현위치로 갱신

            //필요 스텝 수에 맞춰 방향 설정
            if(motor[i].steps.err >= 0) digitalWrite(motor[i].dirPin, HIGH);
            else digitalWrite(motor[i].dirPin, LOW);
            motor[i].steps.err = abs(motor[i].steps.err);       //필요한 스텝 수를 양수로 설정
          }
          //동작에 필요한 최대 스텝 수를 저장하는 maxStep변수 선언
          int Steperr[3] = {0, 0, 0};
          int maxStep = 0;
          //maxStep을 탐색하여 저장
          for(int i = 0; i<NUM_MOTOR; i++)
          {
            //maxStep에 스텝 최대값 저장
            maxStep = (maxStep<motor[i].steps.err)?motor[i].steps.err:maxStep;
            Serial.print("steps: ");
            Serial.print(motor[i].steps.err);
            Serial.println(" ");
            vel[i] = vel[i]*tconst;
          }
          for(int j = 0; j<maxStep; j++)
          { 
            for(int i = 0; i<NUM_MOTOR; i++)
            {
              //sin 곡선 형태로 펄스 생성(del = 간격, var = 속도관련 사인형태변수, 서로 역수관계)
              float var = pow(1 + sin(2 * PI * abs((j+1) - (motor[i].steps.err+1)/2) / (motor[i].steps.err+1) + PI / 2), dim);
              //j번째 펄스에 대해, j가 앞서 계산된 필요 스텝 수보다 작은 경우 펄스 생성
              if(var < multiplier)
              {
                //속도가 0에 수렴할때 최대 delay 값 고정
                var = multiplier;
              }
              if(j<motor[i].steps.err)
              {
                digitalWrite(motor[i].stepPin, HIGH);
                del[i] = 1e+6 / (vel[i] * var);
                delayMicroseconds(del[i]);
              }
              else
              {
                del[i] = del[i] * 0.95;
                delayMicroseconds(del[i]);
              }
              if(var < multiplier)
              {
                //속도가 0에 수렴할때 최대 delay 값 고정
                var = multiplier;
              }
              if(j<motor[i].steps.err)
              {
                digitalWrite(motor[i].stepPin, LOW);
                del[i] = 1e+6 / (vel[i] * var);
                delayMicroseconds(del[i]);
              }
              else
              {
                del[i] = del[i] * 0.95;
                delayMicroseconds(del[i]);
              }
            }
            ///실시간 좌표출력용 코드
            //Serial.print(del[0]*2);
            //Serial.print(" ");
            //Serial.print(del[1]*2);
            //Serial.print(" ");
            //Serial.println(del[2]*2);
          }
        }
      //모드를 갱신
      state.cur = MODE_HOME;
      state.pre = MODE_HOME;
      posX = initX;
      posY = initY;
      posZ = initZ;
      xreceived = false;
      yreceived = false;
      zreceived = false;
      Serial.flush();
      Serial.println("THEEND");                
      }
    }
  } 
  /*------------------------------------------------------------------------------*/

  void homefunction(int i, bool clockwise) {
    ///홈포지션찾기 함수
    int homing_vel = 1000;
    while (digitalRead(motor[i].sensor) == 1)
    { 
      if(clockwise)
      {
        digitalWrite(motor[i].dirPin, HIGH);
      }       
      else
      {
        digitalWrite(motor[i].dirPin, LOW);
      }
      digitalWrite(motor[i].stepPin, HIGH);  
      delayMicroseconds(homing_vel);  
      digitalWrite(motor[i].stepPin, LOW);  
      delayMicroseconds(homing_vel);  
    }
  }

  bool coordrange(int posX, int posY, int posZ)
  {
    if(posX < maxX && posX > minX && posY < maxY && posY > minY && posZ < maxZ && posZ > minZ)
    {
      return true;
    }
    else
    {
      return false;
    }
  }


  float mapping(float x, float minFrom, float maxFrom, float minTo, float maxTo) {
    float value = (x - minFrom) * (maxTo - minTo) / (maxFrom - minFrom) + minTo;
    value = min(max(value, minTo), maxTo);
    return value;
  }


  bool getDegree_byFunction(float X, float Y, float Z, float* deg1, float* deg2, float* deg3) {
    float a1 = 18.5, b1 = 21.5, l11 = 50, l12 = 30, a2 = 57, b2 = 46, l21 = 30, l22 = 50, a3 = 18.5, b3 = 21.5, l31 = 40, l32 = 40;

    float alpha1 = 90, alpha2 = 180, alpha3 = 270;

    alpha1 = deg2rad(alpha1);
    alpha2 = deg2rad(alpha2);
    alpha3 = deg2rad(alpha3);

    float b011 = cos(alpha1) * X + sin(alpha1) * Y - a1 + b1;
    float b021 = -sin(alpha1) * X + cos(alpha1) * Y;
    float b031 = Z;

    float theta31 = acos(b021 / l12);
    float theta21 = acos((b011 * b011 + b021 * b021 + b031 * b031 - l11 * l11 - l12 * l12) / (2 * l11 * l12 * sin(theta31)));

    float g11 = l11 + l12 * cos(theta21) * sin(theta31);
    float g21 = l12 * sin(theta21) * sin(theta31);

    float theta11 = atan2(-g21 * b011 + g11 * b031, g11 * b011 + g21 * b031);

    float b012 = cos(alpha2) * X + sin(alpha2) * Y - a2 + b2;
    float b022 = -sin(alpha2) * X + cos(alpha2) * Y;
    float b032 = Z;

    float theta32 = acos(b022 / l22);
    float theta22 = acos((b012 * b012 + b022 * b022 + b032 * b032 - l21 * l21 - l22 * l22) / (2 * l21 * l22 * sin(theta32)));

    float g12 = l21 + l22 * cos(theta22) * sin(theta32);
    float g22 = l22 * sin(theta22) * sin(theta32);

    float theta12 = atan2(-g22 * b012 + g12 * b032, g12 * b012 + g22 * b032);


    float b013 = cos(alpha3) * X + sin(alpha3) * Y - a3 + b3;
    float b023 = -sin(alpha3) * X + cos(alpha3) * Y;
    float b033 = Z;

    float theta33 = acos(b023 / l32);
    float theta23 = acos((b013 * b013 + b023 * b023 + b033 * b033 - l31 * l31 - l32 * l32) / (2 * l31 * l32 * sin(theta33)));

    float g13 = l31 + l32 * cos(theta23) * sin(theta33);
    float g23 = l32 * sin(theta23) * sin(theta33);

    float theta13 = atan2(-g23 * b013 + g13 * b033, g13 * b013 + g23 * b033);

    *deg1 = rad2deg(theta11);
    *deg2 = rad2deg(theta12);
    *deg3 = rad2deg(theta13);

    return true;
  }

  float deg2rad(float a) {
    return a / 180 * PI;
  }

  float rad2deg(float a) {
    return a / PI * 180;
  }

  int angle2step(float deg) {
    return round(deg / 0.05625);
  }