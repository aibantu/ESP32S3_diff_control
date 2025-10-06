
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "ctrl.h"
//每个服务、特征和描述符都有一个UUID(通用唯一标识符)，UUID用于唯一标识信息，它可以识别蓝牙设备提供的特定服务。
// See the following for generating UUIDs: https://www.uuidgenerator.net/
//
#define BLE_NAME "Bot"
#define SERVICE_UUID  "ba0d1b7e-7ad8-11ef-b864-0242ac120002"
#define CHARACTERISTIC_UUID_RX "c7ebcf24-7ad8-11ef-b864-0242ac120002"
#define CHARACTERISTIC_UUID_TX "d3c5baf8-7ad8-11ef-b864-0242ac120002"

#define MAX_SPEED           (0.6F)  //最大前进速度
#define MAX_YAWSPEED        (0.5F)  // 最大旋转速度


// 限制值在指定范围内的宏
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//蓝牙全局指针捏 先置空防止意外
// BLE相关全局指针
BLEServer *pServer = NULL; // BLE服务器指针
BLEService *pService =NULL; //  // BLE服务指针
BLECharacteristic *pTxCharacteristic =NULL; // 发送特征指针
BLECharacteristic *pRxCharacteristic =NULL; // 接收特征指针

bool deviceConnected = false;                //本次连接状态
// 服务器回调类，处理连接和断开事件
class MyServerCallbacks : public BLEServerCallbacks
{
public:
    // 客户端连接时调用
	void onConnect(BLEServer *server)
	{
		Serial.println("onConnect");
		server->getAdvertising()->stop();// 连接成功后停止广播
        deviceConnected = true;
	}
	void onDisconnect(BLEServer *server)
	{
		Serial.println("onDisconnect");
		server->getAdvertising()->start();// 断开后重新开始广播
        deviceConnected = false;
	}
};

// 特征回调类，处理特征写入事件（接收到客户端数据）
class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)//特征写入事件，收到蓝牙数据
    {
        // 获取原始字节
        uint8_t *data = pCharacteristic->getData(); //接收信息 //getData 返回的是uint8 getValue 返回的是string
        // 获取接收数据的长度（字节数），用于后续校验数据格式是否合法。
        uint32_t len = pCharacteristic->getLength();
        // 声明三个int8_t（8 位有符号整数）变量，
        // 用于临时存储速度、旋转速度、腿长的解析结果（后续未直接使用，可能是预留的中间变量）。
        int8_t speed_t,yawsp_t,length_t;

        // 打印接收到的数据的长度
        Serial.print("接收到的数据长度: ");
        Serial.println(len);

        // 以十六进制形式打印接收到的数据
        Serial.print("接收到的数据(十六进制): ");
        // 循环遍历接收的每个字节，以十六进制形式打印（例如0xA5、0x58）
        for (uint32_t i = 0; i < len; i++) {
            // 若字节值小于0x10（即单个十六进制位，如0x3），则补前导0（显示为03），保证格式统一，便于调试时观察。
            if (data[i] < 0x10) {
                Serial.print("0"); // 确保每个字节都显示为两位十六进制数
            }
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

// 校验帧头：第1字节必须为0xA5（发送端需将首字节固定为0xA5，作为命令起始标识）
// 校验帧尾：第9字节必须为0x5A（发送端需将末字节固定为0x5A，作为命令结束标识）
// 校验长度：数据总长度必须为9字节（发送端需确保发送9字节数据，避免残缺或冗余）
// 校验命令标识：第2字节为0x58（发送端通过该字节指定"站立命令"，与其他命令区分）
// 十六进制的58对应的十进制就是88，因此蓝牙设置standup按钮参数对应数值是88
        if(data[0] == 0xA5 && data[8] == 0x5A && len == 9 && data[1] == 0x58)
        {
            // 二次判断：仅当机器人当前处于"未站立状态"时，才执行站立动作
            // 防止重复触发站立任务（例如发送端连续发送命令时，避免机械冲突）
            
        }
        // 添加跳跃命令识别 - 使用0x5B作为跳跃命令标识
        // 校验命令标识：第6字节为0x5B（发送端通过该字节指定"跳跃命令"，与其他命令区分）
        // 十六进制0x5B对应的十进制是91，
        else if(data[0] == 0xA5 && data[8] == 0x5A && len == 9 && data[5] == 0x5B)
        {
                
        }
        // 校验命令标识：第7字节为0x59（交叉步切换命令的唯一标识）
        // 十六进制的0x59对应十进制89
        else if(data[0] == 0xA5 && data[8] == 0x5A && len == 9 && data[6] == 0x59)
        {
            // 切换交叉步状态
            // crossStepEnabled控制是否启用交叉步模式
            
        }
        // 再处理运动控制命令
        else if(data[0] == 0xA5 && data[8] == 0x5A && len == 9)
        {
            // // 解析前进/后退速度指令（第3字节，索引2）
            // (signed char)data[2]：将无符号字节转为有符号整数（支持正负方向）
            // /100.0f：将原始值归一化到[-1.27, 1.27]范围（127/100=1.27）
            // *MAX_SPEED / 1.27f：将归一化值映射到实际速度范围（0~MAX_SPEED）
            target.speedCmd = ((float)((signed char)data[2]) / 100.0f) * MAX_SPEED / 1.27f;
            // 解析旋转速度指令（第4字节，索引3）
            // 逻辑与速度指令类似，但：
            // - 除以-100.0f：负号用于调整旋转方向（原始值为正→左转，为负→右转）
            // - MAX_YAWSPEED：最大旋转速度（单位推测为rad/s）
            // 示例：若data[3]=50 → 旋转速度=50/-100*0.5/0.4 ≈ -0.625rad/s（左转）
            target.yawSpeedCmd = ((float)((signed char)data[3]) / -100.0f) * MAX_YAWSPEED / 0.4f;
        }


        
    }
};


void BLE_Init(void)
{
    BLEDevice::init(BLE_NAME);      //创建BLE设备

    pServer = BLEDevice::createServer();   //创建BLE服务器
    pServer->setCallbacks(new MyServerCallbacks());   //设置回调函数
    pService = pServer->createService(SERVICE_UUID);   //创建服务

    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX,BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());    //创建并添加描述符
    pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX,BLECharacteristic::PROPERTY_WRITE_NR);   //与 WRITE 属性类似，不同之处在于它不等待服务器的响应
    pRxCharacteristic->setCallbacks(new MyCallbacks()); //设置回调

    pService->start();                  // 开始服务
    pServer->getAdvertising()->start(); // 开始广播
    Serial.println("Waiting for connection \r\n");
}

// txValue 是全局变量，作为要发送的数据缓冲区，初始值为0。
// 每次发送后会递增，形成一个从 0 到 255 循环的计数器。
uint8_t txValue = 0;     //后面需要发送的值
void BLE_TestTask(void *arg)
{

	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
        if (deviceConnected)
        {
            // 第二个参数1：表示发送的数据长度为1 字节（因为txValue是uint8_t类型，占用 1 字节）。
            pTxCharacteristic->setValue(&txValue, 1); 
            pTxCharacteristic->notify();              // 广播
            txValue++;                                // 指针数值自加1
        }
            vTaskDelayUntil(&xLastWakeTime, 100); //100ms轮询
    }
}

void BLE_Test(void)
{
    xTaskCreate(BLE_TestTask, "BLE_TestTask", 4096, NULL, 1, NULL);
}

