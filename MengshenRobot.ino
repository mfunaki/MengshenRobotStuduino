/*
 * STUDUINO
 * (C) 2014 Artec Corporation
 * 
 * 本プログラムをArduino IDEで実行する場合、Studuinoライブラリを
 * インストールしなければなりません。
 * 
 * Studuinoライブラリのインストール方法は、下記のホームページを
 * 参考にしてください。
 * http://www.artec-kk.co.jp/studuino
 * 
 * 下記に主要な関数を説明します
 * --------------------------------------
 * ■ artecRobotSetup関数：接続パーツ等の初期化関数です
 * ■ artecRobotMain関数 ：「制御スタート」ブロックに接続したプログラムです
 * ■ ARSR_*関数         ：「関数ブロック」で作成したプログラムです
 * --------------------------------------
 */
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <MMA8653.h>
#include <MPU6050.h>
#include <IRremoteForStuduino.h>
#include <ColorSensor.h>
#include "Bluetooth.h"
#include "Studuino.h"
#include <avr/pgmspace.h>
// **********************************************************************
// 宣言
// **********************************************************************
// リストの要素
struct cell_t{
  struct cell_t* next;
  float data;
};
typedef cell_t cell;
// **********************************************************************
// 定義
// **********************************************************************
#define DCMPWR(power)	((byte)(min(max(0, ((float)(power) * 2.55)),255)))
#define DCMOTOR_POWER(port, pwr) (dcMotorPower(port, pwr))
#define DCMOTOR_STOP(port, stop) (dcMotorStop(port, stop))
#define BUZZER_START(port, tid) (buzzerStart(port, tid))
#define BUZZER_STOP(port) (buzzerStop(port))


#define BMIN    (0)     // Blockプログラミング環境側のセンサーの最小値
#define BMAX    (100)   // Blockプログラミング環境側のセンサーの最大値
#define ANAMIN  (0)     // Studuino基板のアナログセンサーの最小値
#define ANAMAX  (1023)  // Studuino基板のアナログセンサーの最大値
#define ACCMIN  (-128)  // Studuino基板の加速度センサーの最小値
#define ACCMAX  (127)   // Studuino基板の加速度センサーの最大値
#define GYROMIN	(-32768)	// ジャイロセンサーの最小値
#define GYROMAX	(32767)		// ジャイロセンサーの最大値
#define PUSHSWITCH(port)       (board.GetPushSwitchValue(port))
#define TOUCH_SENSOR(port)     (board.GetTouchSensorValue(port))
#define LIGHT_SENSOR(port)     (map(board.GetLightSensorValue(port), ANAMIN, ANAMAX, BMIN, BMAX))
#define SOUND_SENSOR(port)     (map(board.GetSoundSensorValue(port), ANAMIN, ANAMAX, BMIN, BMAX))
#define IRPHOTOREFLECTOR(port) (map(board.GetIRPhotoreflectorValue(port), ANAMIN, ANAMAX, BMIN, BMAX))
#define ACCELEROMETER(axis)    (map(board.GetAccelerometerValue(axis), ACCMIN, ACCMAX, BMIN, BMAX))

#define ULTRASONIC_SENSOR()    (GetUltrasonicSensorValue())
#define INFRARED_RECEIVER()    (board.GetIRReceiverValue())
#define TEMPERATURE_SENSOR(port)    ((((board.GetTemperatureSensorValue(port) / 1024.0) * 3.3) - 0.5) / 0.01)
#define GYRO_SENSOR(port)    (map(board.GetGyroscopeValue(port), GYROMIN, GYROMAX, BMIN, 1001) / 10.f)
#define COLOR_SENSOR(arg)    (getColorSensorValue(arg))
#define DETECTED_COLOR()    (board.GetColorCode())

#define BLUETOOTH_UPDATE()    (Serial.available())
#define BLUETOOTH_RECVVALUE()    (Serial.read())
#define BLUETOOTH_SENDVALUE(val)    (Serial.write((uint8_t)val))


#define CONTROLLERAPP_UPDATE()    (board.UpdateBluetooth())
#define CONTROLLERAPP_ACCELVALUE(axis)    (map(board.GetBTAccelValue(axis), ACCMIN, ACCMAX, BMIN, BMAX))
#define CONTROLLERAPP_COMMANDID(id)    (board.GetBTCommandIDState(id))

#define DWEIGHT	(1.818)

const byte SQRT  =  (0);   // √n
const byte ABS   =  (1);   // |n|
const byte SIN   =  (2);   // sin(n)
const byte COS   =  (3);   // cos(n)
const byte TAN   =  (4);   // tan(n)
const byte LN    =  (5);   // loge
const byte LOG   =  (6);   // log10
const byte POWE  =  (7);   // e^
const byte POW10 =  (8);   // 10^

PROGMEM const word BTONE[] = {
  BZR_C3,  BZR_CS3, BZR_D3,  BZR_DS3, BZR_E3,  BZR_F3,  BZR_FS3, BZR_G3,  BZR_GS3, BZR_A3,  BZR_AS3, BZR_B3,  
  BZR_C4,  BZR_CS4, BZR_D4,  BZR_DS4, BZR_E4,  BZR_F4,  BZR_FS4, BZR_G4,  BZR_GS4, BZR_A4,  BZR_AS4, BZR_B4,  
  BZR_C5,  BZR_CS5, BZR_D5,  BZR_DS5, BZR_E5,  BZR_F5,  BZR_FS5, BZR_G5,  BZR_GS5, BZR_A5,  BZR_AS5, BZR_B5,  
  BZR_C6,  BZR_CS6, BZR_D6,  BZR_DS6, BZR_E6,  BZR_F6,  BZR_FS6, BZR_G6,  BZR_GS6, BZR_A6,  BZR_AS6, BZR_B6,  
  BZR_C7,  BZR_CS7, BZR_D7,  BZR_DS7, BZR_E7,  BZR_F7,  BZR_FS7, BZR_G7,  BZR_GS7, BZR_A7,  BZR_AS7, BZR_B7,  
  BZR_C8,  
};
#define TONENUM	((sizeof(BTONE)/sizeof(word))-1)
#define BHZ(num)  (pgm_read_word_near(BTONE+(byte)(min(max(0, (num-48)),TONENUM))))

// for Servomotor calibration
const byte SVCD2  = (4);
const byte SVCD4  = (5);
const byte SVCD7  = (6);
const byte SVCD8  = (7);
const byte SVCD9  = (0);
const byte SVCD10 = (1);
const byte SVCD11 = (2);
const byte SVCD12 = (3);
// Min/Max of servomotor's degree
#define SVRANGE(deg)	(min(max(0, (deg)), 180))
#define SYNCSVRANGE(dly)	(min(max(0, (dly)), 20))

const byte VALUE_X = VALUE_CLEAR + 1;
const byte VALUE_Y = VALUE_CLEAR + 2;

// **********************************************************************
// プロトタイプ宣言
// **********************************************************************
// リスト
int listDelete(struct cell_t* p, int pos);              // リストの要素を削除
int listAdd(struct cell_t* p, float data);              // リストに要素を追加
int listInsert(struct cell_t *p, int pos, float data);  // リストに要素を挿入
int listReplace(struct cell_t *p, int pos, float data); // リストの要素を置き換える
int listLength(struct cell_t *p);                       // リストの長さを取得
float listItem(struct cell_t *p, int pos);              // リストの要素を取得
bool listIsContain(struct cell_t *p, float data);       // リストにデータが含まれるかどうかの確認
// 丸め処理
int scratchRound(float arg);
// 数学処理
float math(byte opeID, float arg);              // 算術処理
// タイマー処理関数
void resetTimer();
float getTimer();
// 温度センサーノイズ対策
int averageTemperature(byte pin);

// ロボットセットアップ処理
void artecRobotSetup();
// ロボットメイン処理
void artecRobotMain();

// DCモーター処理
void dcMotorPower(byte port, byte pwr);
void dcMotorStop(byte port, byte stop);

// 超音波センサー、赤外線リモコン受信との併用対応
float GetUltrasonicSensorValue();

// ブザー
void buzzerStart(byte port, byte tid);
void buzzerStop(byte port);

// カラーセンサー
double getColorSensorValue(byte type);
// **********************************************************************
// グローバル変数
// **********************************************************************
Studuino board;	// Studuino基板オブジェクト
unsigned long StartTime;	// For timer
bool IRRemoteUsed = false;	// 赤外線受信センサ使用中フラグ
bool BeepOn = false;
bool DCMotorOn = false;

// **********************************************************************
// プログラム
// **********************************************************************

// ---------------------------------------
// Servomotor calibration data
// ---------------------------------------
char SvCalibrationData[] = { 0, 0, 0, 0, 0, 0, 0, 0,  };
// ---------------------------------------
// DC motor calibration data
// ---------------------------------------
byte DCCalibrationData[] = { 100, 100 };
// ---------------------------------------
// prototype declaration
// ---------------------------------------
void ARSR_CommandSpeak03();
void ARSR_CommandSpeak02();
void ARSR_CommandSpeak01();
void ARSR_InitPos();
void ARSR_TurnOffLED();
void ARSR_TurnOnLED();
void ARSR_InitLED();
void ARSR_InitHome();
void artecRobotMain();
void ARSR_localScriptStart();
void ARSR_CommandInit();
void ARSR_SetToneSub();
// ---------------------------------------
// Global variable
// ---------------------------------------
float ARVAL_D11X;
float ARVAL_D11XMax;
float ARVAL_D11Y;
float ARVAL_D11YMax;
float ARVAL_D7X;
float ARVAL_D7XMax;
float ARVAL_D7Y;
float ARVAL_D7YMax;
float ARVAL_D9X;
float ARVAL_D9XMax;
float ARVAL_D9Y;
float ARVAL_D9YMax;
float ARVAL_PrevTone;
float ARVAL_Tone;
float ARVAL_X;
float ARVAL_XMax;
float ARVAL_Y;
float ARVAL_YMax;
float ARVAL_isProcessed;
cell ARLIST_commandList;
byte port[8];
byte degree[8];
// ---------------------------------------
// Artec robot setup routine
// ---------------------------------------
void artecRobotSetup() {
    board.InitServomotorPort(PORT_D7);
    board.InitServomotorPort(PORT_D8);
    board.InitServomotorPort(PORT_D9);
    board.InitServomotorPort(PORT_D10);
    board.InitServomotorPort(PORT_D11);
    board.InitServomotorPort(PORT_D12);
    board.InitSensorPort(PORT_A0, PIDLED);
    board.InitSensorPort(PORT_A1, PIDLED);
    board.InitSensorPort(PORT_A2, PIDLED);
    board.InitSensorPort(PORT_A3, PIDLED);
    board.InitBluetooth();
    board.SetServomotorCalibration(SvCalibrationData);
    board.SetDCMotorCalibration(DCCalibrationData);
}
// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------
void ARSR_CommandSpeak03() {
    if ( (!(ARVAL_D7YMax==90)) ) { 
        
        port[0] = PORT_D7;	degree[0] = SVRANGE(ARVAL_D7YMax);
        port[1] = PORT_D8;	degree[1] = SVRANGE(ARVAL_D7XMax);
        board.SyncServomotors(port, degree, 2, SYNCSVRANGE(scratchRound(20-20))+3);
    }
}

// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------
void ARSR_CommandSpeak02() {
    if ( (!(ARVAL_D11YMax==90)) ) { 
        
        port[0] = PORT_D11;	degree[0] = SVRANGE(ARVAL_D11YMax);
        port[1] = PORT_D12;	degree[1] = SVRANGE(ARVAL_D11XMax);
        board.SyncServomotors(port, degree, 2, SYNCSVRANGE(scratchRound(20-20))+3);
    }
}

// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------
void ARSR_CommandSpeak01() {
    if ( (!(ARVAL_D9YMax==90)) ) { 
        
        port[0] = PORT_D9;	degree[0] = SVRANGE(ARVAL_D9YMax);
        port[1] = PORT_D10;	degree[1] = SVRANGE(ARVAL_D9XMax);
        board.SyncServomotors(port, degree, 2, SYNCSVRANGE(scratchRound(20-20))+3);
    }
}

// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------
void ARSR_InitPos() {
    
    port[0] = PORT_D9;	degree[0] = SVRANGE(ARVAL_D9Y);
    port[1] = PORT_D10;	degree[1] = SVRANGE(ARVAL_D9X);
    port[2] = PORT_D11;	degree[2] = SVRANGE(ARVAL_D11Y);
    port[3] = PORT_D12;	degree[3] = SVRANGE(ARVAL_D11X);
    port[4] = PORT_D7;	degree[4] = SVRANGE(ARVAL_D7Y);
    port[5] = PORT_D8;	degree[5] = SVRANGE(ARVAL_D7X);
    board.SyncServomotors(port, degree, 6, SYNCSVRANGE(scratchRound(20-20))+3);
}

// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------
void ARSR_TurnOffLED() {
    board.LED(PORT_A0,OFF);
    board.LED(PORT_A1,OFF);
    board.LED(PORT_A2,OFF);
    board.LED(PORT_A3,OFF);
}

// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------
void ARSR_TurnOnLED() {
    board.LED(PORT_A0,ON);
    board.LED(PORT_A1,ON);
    board.LED(PORT_A2,ON);
    board.LED(PORT_A3,ON);
}

// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------
void ARSR_InitLED() {
    ARSR_TurnOnLED();
    board.Timer(1);
    ARSR_TurnOffLED();
}

// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------
void ARSR_InitHome() {
    ARVAL_X=0;
    ARVAL_Y=90;
    
    port[0] = PORT_D9;	degree[0] = SVRANGE(ARVAL_Y);
    port[1] = PORT_D10;	degree[1] = SVRANGE(ARVAL_X);
    port[2] = PORT_D11;	degree[2] = SVRANGE(ARVAL_Y);
    port[3] = PORT_D12;	degree[3] = SVRANGE(ARVAL_X);
    port[4] = PORT_D7;	degree[4] = SVRANGE(ARVAL_Y);
    port[5] = PORT_D8;	degree[5] = SVRANGE(ARVAL_X);
    board.SyncServomotors(port, degree, 6, SYNCSVRANGE(scratchRound(20-0))+3);
}

// ---------------------------------------
// Artec robot mainroutine
// ---------------------------------------
void artecRobotMain() {
    ARSR_InitHome();
    ARSR_InitLED();
    for (;;) {  
        ARSR_localScriptStart();
        board.Timer(0.1);
    }
}

// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------
void ARSR_localScriptStart() {
    if ( BLUETOOTH_UPDATE() ) { 
        ARSR_TurnOnLED();
        listAdd(&ARLIST_commandList,BLUETOOTH_RECVVALUE());
        ARSR_TurnOffLED();
    }
    if ( (listLength(&ARLIST_commandList)==1) ) { 
        board.LED(PORT_A0,ON);
    }
    if ( (listLength(&ARLIST_commandList)==2) ) { 
        board.LED(PORT_A1,ON);
    }
    if ( (listLength(&ARLIST_commandList)==3) ) { 
        board.LED(PORT_A2,ON);
    }
    if ( (listLength(&ARLIST_commandList)==4) ) { 
        board.LED(PORT_A3,ON);
        ARSR_TurnOffLED();
        ARVAL_isProcessed=0;
        if ( (listItem(&ARLIST_commandList,1)==73) ) { 
            board.LED(PORT_A3,ON);
            ARSR_CommandInit();
            ARSR_InitPos();
            ARVAL_isProcessed=1;
        }
        if ( (listItem(&ARLIST_commandList,1)==65) ) { 
            board.LED(PORT_A0,ON);
            ARSR_CommandSpeak01();
            ARVAL_isProcessed=1;
        }
        if ( (listItem(&ARLIST_commandList,1)==66) ) { 
            board.LED(PORT_A1,ON);
            ARSR_CommandSpeak02();
            ARVAL_isProcessed=1;
        }
        if ( (listItem(&ARLIST_commandList,1)==67) ) { 
            board.LED(PORT_A2,ON);
            ARSR_CommandSpeak03();
            ARVAL_isProcessed=1;
            board.LED(PORT_A2,OFF);
        }
        if ( (ARVAL_isProcessed==1) ) {  
            listDelete(&ARLIST_commandList,1);
            listDelete(&ARLIST_commandList,1);
            listDelete(&ARLIST_commandList,1);
            listDelete(&ARLIST_commandList,1);
        } else {
            listDelete(&ARLIST_commandList,1);
        }
    }
}

// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------
void ARSR_CommandInit() {
    ARVAL_PrevTone=0;
    ARVAL_Tone=listItem(&ARLIST_commandList,2);
    ARSR_SetToneSub();
    ARVAL_D9X=ARVAL_X;
    ARVAL_D9XMax=ARVAL_XMax;
    ARVAL_D9Y=ARVAL_Y;
    ARVAL_D9YMax=ARVAL_YMax;
    ARVAL_Tone=listItem(&ARLIST_commandList,3);
    ARSR_SetToneSub();
    ARVAL_D11X=ARVAL_X;
    ARVAL_D11XMax=ARVAL_XMax;
    ARVAL_D11Y=ARVAL_Y;
    ARVAL_D11YMax=ARVAL_YMax;
    ARVAL_Tone=listItem(&ARLIST_commandList,4);
    ARSR_SetToneSub();
    ARVAL_D7X=ARVAL_X;
    ARVAL_D7XMax=ARVAL_XMax;
    ARVAL_D7Y=ARVAL_Y;
    ARVAL_D7YMax=ARVAL_YMax;
}

// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------
void ARSR_SetToneSub() {
    ARVAL_X=0;
    ARVAL_XMax=60;
    ARVAL_Y=90;
    ARVAL_YMax=90;
    if ( (ARVAL_Tone==49) ) { 
        ARVAL_Y=135;
        ARVAL_YMax=135;
    }
    if ( (ARVAL_Tone==50) ) { 
        ARVAL_Y=60;
        ARVAL_YMax=135;
    }
    if ( (ARVAL_Tone==51) ) { 
        ARVAL_Y=50;
        ARVAL_YMax=50;
    }
    if ( (ARVAL_Tone==52) ) { 
        ARVAL_Y=135;
        ARVAL_YMax=60;
    }
    if ( (ARVAL_Tone==48) ) { 
        ARVAL_XMax=30;
        if ( (ARVAL_PrevTone==49) ) { 
            ARVAL_Y=80;
            ARVAL_YMax=80;
        }
        if ( (ARVAL_PrevTone==50) ) { 
            ARVAL_Y=80;
            ARVAL_YMax=80;
        }
        if ( (ARVAL_PrevTone==51) ) { 
            ARVAL_Y=120;
            ARVAL_YMax=120;
        }
        if ( (ARVAL_PrevTone==52) ) { 
            ARVAL_Y=40;
            ARVAL_YMax=40;
        }
    }
    ARVAL_PrevTone=ARVAL_Tone;
}


// --------------------------------------------
// 概要    : セットアップ処理
// --------------------------------------------
void setup() {
    randomSeed(analogRead(0));
    resetTimer();
    artecRobotSetup();
    artecRobotMain();
}

void loop() {}

// --------------------------------------------
// 概要    : リストからの削除処理
// 引数    : struct _cell_t *p  リストのポインタ
//         : int pos            リストから削除する位置
// 戻り値  : 成功：0, エラー：-1
// --------------------------------------------
int listDelete(struct cell_t* p, int pos)
{
  // 削除位置が0以下の場合、エラーを返す(何もしない)
  if (pos <= 0) { return (-1); }
  // 削除位置がリストの長さよりも大きい場合、エラーを返す(何もしない)
  int l = listLength(p);        // リスト長を取得
  if (l < pos) { return (-1); } 

  cell_t *target, *before;      // 削除する要素とその前の要素
  target = p->next;             // 先頭の次の要素を設定
  before = NULL;
  if (target == NULL) return (-1);  // 既に削除する要素がない場合、エラーを返す
  // 削除対象となる要素に移動する
  before = p;
  for (int i = 0;i < pos-1;i++) {
    if (target->next == NULL) return (-1);  // 削除対象となる要素がない場合、エラーを返す
    before = target;        // 削除対象となる要素の一つ前の要素を退避
    target = target->next;  // 削除対象となる要素の更新
  }
  // 削除対象となる要素が存在する場合
  before->next = target->next;  // 対象の一つ前の要素に対象の次の要素を設定
  delete target;  // 対象を削除
  return(0);
}

// --------------------------------------------
// 概要    : リストへの追加処理
// 引数    : struct _cell_t *p  リストのポインタ
//         : int    data        追加データ
// 戻り値  : 成功：0, エラー：-1
// --------------------------------------------
int listAdd(struct cell_t* p, float data)
{
  cell_t *elm, *last;
  // リスト要素の確保
  elm = new cell_t;
  // 要素の確保に失敗した場合
  if(elm == NULL) {
    // エラーを返す
    return(-1);
  }
  // lastにリストの終端を設定
  last = p;
  for (;;) {
    if (last->next == NULL) break;
    last = last->next;
  }
  // リストの終端に追加する要素の設定
  elm->data = data;
  elm->next = NULL;
  last->next = elm;
  return(0);
}

// --------------------------------------------
// 概要    : リスト長の取得
// 引数    : struct _cell_t *p  リストのポインタ
// 戻り値  : リスト長
// --------------------------------------------
int listLength(struct cell_t* p)
{
  struct cell_t *last;
  // リストの終端に移動
  last = p;
  int length = 0;
  for (;;) {
    if (last->next == NULL) break;
    last = last->next;
    length++;
  }
  // リストの終端に追加する要素の設定
  return(length);
}

// --------------------------------------------
// 概要    : リスト要素の取得
// 引数    : struct _cell_t *p  リストのポインタ
//         : int    pos         リスト要素の取得位置
// 戻り値  : リスト要素、要素が存在しない場合は、0を返す
// --------------------------------------------
float listItem(struct cell_t *p, int pos)
{
  // 取得位置が0以下の場合、0を返す
  if (pos <= 0) { return (0); }
  // 取得位置がリストの長さよりも大きい場合、0を返す
  int l = listLength(p);    // リスト長を取得
  if (l < pos) { return (0); }

  struct cell_t *target;    // 取得する要素
  target = p;               // 先頭の要素を設定
  // 取得対象となる要素に移動する
  for (int i = 0;i < pos;i++) {
    target = target->next;  // 取得対象となる要素の更新
  }
  return target->data;
}

// --------------------------------------------
// 概要    : リストへの挿入処理
// 引数    : struct _cell_t *p   リストのポインタ
//         : int pos             挿入する位置
//         : float data          挿入するデータ
// 戻り値  : 成功：0, エラー：-1
// --------------------------------------------
int listInsert(struct cell_t *p, int pos, float data)
{
  // 挿入位置が0以下の場合、エラーを返す(何もしない)
  if (pos <= 0) { return (-1); }
  // 挿入位置がリストの長さ+1よりも大きい場合、エラーを返す(何もしない)
  int l = listLength(p);  // リスト長を取得
  if (l+1 < pos) { return (-1); } 
  // 挿入位置がリストの終端の場合
  if (l+1 == pos) {
    // リストの終端に追加する
    listAdd(p, data);
    return (0);
  }

  struct cell_t *item, *target, *before;	// 挿入する要素、挿入する位置の要素とその前の要素
  // リスト要素の確保
  item = new cell_t;
  // 要素の確保に失敗した場合、エラーを返す(何もしない)
  if(item == NULL) { return(-1); }

  target = p;
  // 挿入対象となる要素に移動する
  for (int i = 0;i < pos;i++) {
    before = target;        // 挿入対象となる要素の一つ前の要素を退避
    target = target->next;  // 挿入対象となる要素の更新
  }
  // 挿入対象となる要素が存在する場合
  item->data = data;    // 要素のデータの設定
  item->next = target;  // 次の要素を設定
  before->next = item;  // 1つ前の要素に対象の次の要素を設定
  return(0);
}

// --------------------------------------------
// 概要    : リストの要素の置換処理
// 引数    : struct _cell_t *p  リストのポインタ
//         : int pos            置換する位置
//         : float data         置換するデータ
// 戻り値  : 成功：0, エラー：-1
// --------------------------------------------
int listReplace(struct cell_t *p, int pos, float data)
{
  // 置換位置が0以下の場合、エラーを返す(何もしない)
  if (pos <= 0) { return (-1); }
  // 置換位置がリストの長さよりも大きい場合、エラーを返す(何もしない)
  int l = listLength(p);  // リスト長を取得
  if (l < pos) { return (-1); } 

  struct cell_t *target;  // 置換する要素

  target = p;
  // 置換対象となる要素に移動する
  for (int i = 0;i < pos;i++) {
    target = target->next;  // 置換対象となる要素の更新
  }
  // 置換対象となる要素が存在する場合
  target->data = data;      // 要素のデータの設定
  return(0);
}

// --------------------------------------------
// 概要    : リストの要素に指定データが存在するか？
// 引数    : struct _cell_t *p  リストのポインタ
//         : float data         検索するデータ
// 戻り値  : 存在する：true, 存在しない：false
// --------------------------------------------
bool listIsContain(struct cell_t *p, float data)
{
  struct cell_t *elm = p;
  // リストの全要素に対してdataを検索する
  for (;;) {
    // リスト終端に到達したらbreak
    if (elm->next == NULL) break;
    // リストの次の要素を取得
    elm = elm->next;
    // リストにdataが存在する場合、trueを返す
    if (elm->data == data) return true;
  }
  // リストにdataが存在しない場合、falseを返す
  return false;
}

// --------------------------------------------
// 概要    : 丸め処理
//         : float  arg    引数
// 戻り値  : 演算結果
// --------------------------------------------
int scratchRound(float arg)
{
  return round(arg);
}

// --------------------------------------------
// 概要    : 算術演算処理
// 引数    : byte   opeID  操作ID
//         : float  arg    引数
// 戻り値  : 演算結果
// --------------------------------------------
float math(byte opeID, float arg)              // 算術処理
{
  float result;
  switch (opeID) {
    case SQRT:
      result = sqrt(arg);
    break;
    case ABS:     // |n|
      result = abs(arg);
    break;
    case SIN:     // sin(n)
    {
      float rad = arg * PI / 180.0;
      result = sin(rad);
    }
    break;
    case COS:     // cos(n)
    {
      float rad = arg * PI / 180.0;
      result = cos(rad);
    }
    break;
    case TAN:     // tan(n)
    {
      float rad = arg * PI / 180.0;
      result = tan(rad);
    }
    break;
/*
    case ASIN:    // arcsin(n)
    case ACOS:    // arccos(n)
    case ATAN:    // arctan(n)
    break;
*/
    case LN:      // loge
      result = log(arg);
    break;
    case LOG:     // log10
      result = log10(arg);
    break;
    case POWE:    // e^
      result = exp(arg);
    break;
    case POW10:   // 10^
      result = pow(10, arg);
    break;
    default:
      result = 0;
    break;
    
  }
  return result;
}

// --------------------------------------------
// 概要    : タイマー値の取得
// 戻り値  : タイマーの値(sec)
// --------------------------------------------
float getTimer()
{
  return ((millis() - StartTime) / 1000.0);
}

// --------------------------------------------
// 概要    : タイマー値のリセット
// --------------------------------------------
void resetTimer()
{
  StartTime = millis();
}

// ---------------------------------------------------------------------
// 概要    : 超音波センサーの値を取得
// ---------------------------------------------------------------------
float GetUltrasonicSensorValue() {
	float Distance = board.GetUltrasonicSensorValue(PORT_A0, PORT_A1);
	// ビープ処理中 or DCモーター回転中ではない場合
	if (IRRemoteUsed) {
		if (!(BeepOn | DCMotorOn)) {
			Distance = Distance * DWEIGHT;
		}
	}
	Distance = Distance / 58.0;
	return min(Distance,400);
}


// ---------------------------------------------------------------------
// 概要    : DCモーターのパワーを設定する
// ---------------------------------------------------------------------
void dcMotorPower(byte port, byte pwr) {
	board.DCMotorPower(port, DCMPWR(pwr));
}

// ---------------------------------------------------------------------
// 概要    : DCモーターを停止する
// ---------------------------------------------------------------------
void dcMotorStop(byte port, byte stop) {
	board.DCMotorControl(port, stop);
}

// ---------------------------------------------------------------------
// 概要    : ブザーから音を出力する
// ---------------------------------------------------------------------
void buzzerStart(byte port, byte tid) {
	board.BuzzerControl(port, ON, BHZ(tid));
}

// ---------------------------------------------------------------------
// 概要    : ブザーを停止する
// ---------------------------------------------------------------------
void buzzerStop(byte port) {
	board.BuzzerControl(port, OFF, 0);
}

