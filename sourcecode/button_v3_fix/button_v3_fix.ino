  #define NUM_MOTOR 3         //모터의 수
  
  //호밍 상태 정의 상수
  #define MODE_HOME 0         //동작모드 : 홈 위치
  #define MODE_MOVE 1         //동작모드 : 사용자 지정 동작

  //홈 위치 지정
  int POS_HOME[3] = {1167, 950, 1130};

  //모터드라이버 enable핀 LOW->활성화, HIGH->비활성화
  #define MOTOR_ON LOW        //모터드라이버 활성화
  #define MOTOR_OFF HIGH      //모터드라이버 비활성화

  //상태를 의미하는 구조체. 현재(cur), 이전값(pre), 차이(err)
  typedef struct {
    int cur;
    int pre;
    int err;
  } states;

  //핀 번호 모음
  int SW1 = 2;
  int SW2 = 3;
  int SW3 = 4;
  int SW4 = 5;
  int SW5 = 6;
  int SW6 = 7;
  int SWH = 8;
  int dir[NUM_MOTOR] = {48, 36, 26};
  int step[NUM_MOTOR] = {46, 34, 24};
  int sensor[NUM_MOTOR]= {44, 32, 22};

  //스텝모터를 표현하는 구조체. 방향핀(dirPin), 스텝핀(stepPin), 호밍센서(sensor), 스텝정보(steps. 현재, 이전, 오차)
  typedef struct {
    int dirPin;
    int stepPin;
    int sensor;
    states steps;
    //모터의 경우 cur은 목적 위치, pre는 현재 위치, err는 목적위치로 가기위한 스텝 수를 의미
  } stepMotor;

  //err 값 설정
  int err[3] = {0, 0, 0};

  //스텝모터 핀 및 홈 위치 설정
  stepMotor motor[NUM_MOTOR] = {
    dir[0], step[0], sensor[0],
    { -POS_HOME[0], -POS_HOME[0], 0},
    dir[1], step[1], sensor[1],
    { -POS_HOME[1], -POS_HOME[1], 0},
    dir[2], step[2], sensor[2],
    { POS_HOME[2], POS_HOME[2], 0}
  };

  //모드 초기화
  states state = {MODE_HOME, MODE_HOME, MODE_HOME};

  //Inverse Kinematics 판별 후 좌표 입력 함수
  bool isDataexist();
  bool coordrange();
  void coordprint();
  void homefunction();

  //호밍 함수
  bool homestart = false;
  bool homeend = false;

  //좌표 입력여부 판별
  bool buttonpressed = false;
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

  ///좌표변수
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

  void setup() {
    //시리얼 통신 설정
    Serial.begin(9600);
    for (int i = 0; i < NUM_MOTOR; i++)
    {
      pinMode(motor[i].dirPin, OUTPUT);
      pinMode(motor[i].stepPin, OUTPUT);
      pinMode(motor[i].sensor, INPUT);
    }
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);
    pinMode(SW3, INPUT_PULLUP);
    pinMode(SW4, INPUT_PULLUP);
    pinMode(SW5, INPUT_PULLUP);
    pinMode(SW6, INPUT_PULLUP);
    pinMode(SWH, INPUT_PULLUP);
  }

  void loop() {
    //동작을 시작
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
      state.pre = MODE_MOVE;
      Serial.flush();
      delay(500);
    }

    //버튼입력 시작
    while(state.pre && !state.cur)
    {
      //버튼 입력 1개 판별
      while(!buttonpressed)
      {
        if(digitalRead(SW6) == LOW)
        {
          Serial.println('X');
          posX = posX + 1;
          buttonpressed = true;
        }
        else
        if(digitalRead(SW4) == LOW)
        {
          Serial.println('Y');
          posY = posY + 1;
          buttonpressed = true;
        }
        else
        if(digitalRead(SW2) == LOW)
        {
          Serial.println('Z');
          posZ = posZ + 1;
          buttonpressed = true;
        }
        else
        if(digitalRead(SW5) == LOW)
        {
          Serial.println('x');
          posX = posX - 1;
          buttonpressed = true;
        }
        else
        if(digitalRead(SW3) == LOW)
        {
          Serial.println('y');
          posY = posY - 1;
          buttonpressed = true;
        }
        else
        if(digitalRead(SW1) == LOW)
        {
          Serial.println('z');
          posZ = posZ - 1;
          buttonpressed = true;
        }
        else
        if(digitalRead(SWH) == LOW)
        {
          Serial.println('o');
          posX = 0;
          posY = 0;
          posZ = 46;
          buttonpressed = true;
        }
        /*else
        {
          delay(1000);버튼 누르는 시간관련 딜레이
        }*/
      }

      if(coordrange(posX, posY, posZ))
      {
        Serial.println("Coordinates in range.");
        state.cur = MODE_MOVE;
        prevX = posX;
        prevY = posY;
        prevZ = posZ;
      }
      //좌표가 범위 밖인 경우 처음부터 재입력
      else
      {
        Serial.println("Coordinates out of range!");
        buttonpressed = false;
        posX = prevX;
        posY = prevY;
        posZ = prevZ;
      }
    }

    delay(100);///

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
      buttonpressed = false;           
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
