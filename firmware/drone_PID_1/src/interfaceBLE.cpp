#include "interfaceBLE.h"

NimBLECharacteristic *pTxChar = nullptr;

CmdtBLE cmdt;
bool writeCheckOK = false;
float max_tilt_RPdeg = 15.0;
float max_tilt_Ydegr = 15.0;
float max_throttle_per = 10.0;
float expo = 0.5;

class CmdtCallback : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo& connInfo)
    {
        NimBLEAttValue raw = pChar->getValue();
        //Serial.println("WRITE CALLED"); 

        uint8_t type = raw[0]; // 先頭1バイトでコマンド判定

        switch(type) //'W', 'S'. 'D', 'X', 'C', 'H'
        {
            // W ライトチェックコマンド
            case 'W':
                writeCheckOK = true;
                Serial.println("W RECEIVED");
                break;
            
            // S スタートコマンド
            case 'S':
                if(raw.size() >= 2) 
                {
                    cmdt.startFlag = raw[1];
                    Serial.println("S RECEIVED");
                }
                break;

            // D デフォルトコマンド
            case 'D':
                if(raw.size() >= 2) 
                {
                    cmdt.defaultFlag = raw[1];
                    Serial.println("D RECEIVED");
                }
                break;

            // X 制御切り替えコマンド
            case 'X':
                if(raw.size() >= 2) 
                {
                    cmdt.w_to_s_Flag = raw[1];
                    Serial.println("X RECEIVED");
                }
                break;

            // C 操作コマンド
            case 'C':
                if(raw.size() >= 5)
                { // type + 4 unit8_t
                    cmdt.roll_tag     = raw[1];
                    cmdt.pitch_tag    = raw[2];
                    cmdt.yaw_rate_tag = raw[3];
                    cmdt.throttle_cmd = raw[4];
                    cmdt.LastCmdTime  = millis();
                }
                break;
            
            // H スロットルホバーPIDコマンド
            case 'H':
                int len = raw.length();
                Serial.print("CRITICAL CHECK - Received Bytes: ");
                Serial.println(len);

                for(int i=0; i<raw.length(); i++)
                {
                    Serial.print(raw[i]); 
                    Serial.print(" ");
                }
                Serial.println();Serial.println();

                if(raw.size() >= 20)
                { // type + 10 uint8_t
                    cmdt.throttle_hover = raw[1];
                    cmdt.Kp_roll        = raw[2];
                    cmdt.Ki_roll        = raw[3];
                    cmdt.Kd_roll        = raw[4];
                    cmdt.Kp_pitch       = raw[5];
                    cmdt.Ki_pitch       = raw[6];
                    cmdt.Kd_pitch       = raw[7];
                    cmdt.Kp_yaw         = raw[8];
                    cmdt.Ki_yaw         = raw[9];
                    cmdt.Kd_yaw         = raw[10];
                    cmdt.Kp_roll_rate   = raw[11];
                    cmdt.Ki_roll_rate   = raw[12];
                    cmdt.Kd_roll_rate   = raw[13];
                    cmdt.Kp_pitch_rate  = raw[14];
                    cmdt.Ki_pitch_rate  = raw[15];
                    cmdt.Kd_pitch_rate  = raw[16];
                    cmdt.Kp_yaw_rate    = raw[17];
                    cmdt.Ki_yaw_rate    = raw[18];
                    cmdt.Kd_yaw_rate    = raw[19];
                    Serial.println("H RECEIVED");
                }
                break;
        }   
    }
};

void initBLE()
{
    NimBLEDevice::init("DroneFC");
    NimBLEServer *pServer = NimBLEDevice::createServer();
    NimBLEService *pService = pServer->createService("0000A000-0000-1000-8000-00805F9B34FB");
    
    NimBLECharacteristic *pChar = pService->createCharacteristic(
        "0000A001-0000-1000-8000-00805F9B34FB",
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::READ
    );

    pTxChar = pService->createCharacteristic(
        "0000A002-0000-1000-8000-00805F9B34FB",
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ 
    );

    pChar->setCallbacks(new CmdtCallback());
    pService->start();

// --- アドバタイズ設定の修正（軽量化） ---
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    
    // アドバタイズデータ（メインのパケット）
    NimBLEAdvertisementData advData;
    advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    advData.setCompleteServices(NimBLEUUID("0000A000-0000-1000-8000-00805F9B34FB"));
    // 名前はここに入れない（スキャンレスポンスに任せる）

    // スキャンレスポンス（スマホが「これ何？」と聞いた時に返すパケット）
    NimBLEAdvertisementData scanResponseData;
    scanResponseData.setName("DroneFC"); // 名前はこっちだけに入れる
    
    pAdvertising->setAdvertisementData(advData);
    pAdvertising->setScanResponseData(scanResponseData);
    
    pAdvertising->start();
    Serial.println("BLE Advertising simplified.");
}

void updateBLEPID(const CmdtBLE &Cmdt, PIDCtrl &Pid)
{
    Pid.throttle_hover = Cmdt.throttle_hover / 100.0;

    if(cmdt.w_to_s_Flag) //単ループ制御時
    {
        Pid.Kp_roll        = Cmdt.Kp_roll  / 10000.0;
        Pid.Kp_pitch       = Cmdt.Kp_pitch / 10000.0;
        Pid.Kp_yaw         = Cmdt.Kp_yaw   / 10000.0;
    }
    else                 //カスケード制御時
    {
        Pid.Kp_roll        = Cmdt.Kp_roll  / 10.0;
        Pid.Kp_pitch       = Cmdt.Kp_pitch / 10.0;
        Pid.Kp_yaw         = Cmdt.Kp_yaw   / 10.0;
    }
    Pid.Ki_roll        = Cmdt.Ki_roll  / 10000.0;
    Pid.Kd_roll        = Cmdt.Kd_roll  / 10000.0;

    Pid.Ki_pitch       = Cmdt.Ki_pitch / 10000.0;
    Pid.Kd_pitch       = Cmdt.Kd_pitch / 10000.0;

    Pid.Ki_yaw         = Cmdt.Ki_yaw   / 10000.0;
    Pid.Kd_yaw         = Cmdt.Kd_yaw   / 10000.0;

    Pid.Kp_roll_rate   = Cmdt.Kp_roll_rate  / 10000.0;
    Pid.Ki_roll_rate   = Cmdt.Ki_roll_rate  / 10000.0;
    Pid.Kd_roll_rate   = Cmdt.Kd_roll_rate  / 10000.0;

    Pid.Kp_pitch_rate  = Cmdt.Kp_pitch_rate / 10000.0;
    Pid.Ki_pitch_rate  = Cmdt.Ki_pitch_rate / 10000.0;
    Pid.Kd_pitch_rate  = Cmdt.Kd_pitch_rate / 10000.0;

    Pid.Kp_yaw_rate    = Cmdt.Kp_yaw_rate   / 10000.0;
    Pid.Ki_yaw_rate    = Cmdt.Ki_yaw_rate   / 10000.0;
    Pid.Kd_yaw_rate    = Cmdt.Kd_yaw_rate   / 10000.0;
}

void updateBLECMD(const CmdtBLE &Cmdt, CmdrBLE &Cmdr)
{
    float expo_rtag   = (1 - expo)*((Cmdt.roll_tag  - 100.0)    / 100.0) + expo*((Cmdt.roll_tag  - 100.0)    / 100.0)*((Cmdt.roll_tag  - 100.0)    / 100.0)*((Cmdt.roll_tag  - 100.0)    / 100.0);
    float expo_ptag   = (1 - expo)*((Cmdt.pitch_tag - 100.0)    / 100.0) + expo*((Cmdt.pitch_tag - 100.0)    / 100.0)*((Cmdt.pitch_tag - 100.0)    / 100.0)*((Cmdt.pitch_tag - 100.0)    / 100.0);
    float expo_yrtag  = (1 - expo)*((Cmdt.yaw_rate_tag - 100.0) / 100.0) + expo*((Cmdt.yaw_rate_tag - 100.0) / 100.0)*((Cmdt.yaw_rate_tag - 100.0) / 100.0)*((Cmdt.yaw_rate_tag - 100.0) / 100.0);

    Cmdr.roll_tag      = expo_rtag  * max_tilt_RPdeg;
    Cmdr.pitch_tag     = expo_ptag  * max_tilt_RPdeg;
    Cmdr.yaw_rate_tag  = expo_yrtag * max_tilt_Ydegr;
    Cmdr.throttle_cmd  = ((Cmdt.throttle_cmd - 100.0) / 100.0) * (max_throttle_per / 100.0);
}

void sendDebug(String msg)
{
    if(pTxChar)
    {
        pTxChar->setValue(msg.c_str());
        pTxChar->notify();
    }
}
