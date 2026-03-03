应急DR主控板（无线版本）的源码。  

**说明**  

BW16提供的arduino工程源码（V3.1.9版本）中，WiFiClient类的实现有缺陷，与文档描述不符：  

文档中关于WiFiServer::available的描述为：“Gets a client that is connected to the server and has data available for reading. The connection persists when the returned client object goes out of scope; ....”  

实际情况是，WiFiServer::available返回的WiFiClient对象在WiFiServer::available函数离开调用作用域后，会释放已建立的连接（WiFiClient的析构函数中会调用stop）。这导致Non-Bolcking的TCP server无法工作。  

amebaiot论坛上有人反馈过这个问题：https://forum.amebaiot.com/t/wifiserver-wificlient-connection-doesnt-persist-when-client-object-goes-out-of-scope/3177  
维护者答复此问题已经在ameba-arduino-pro2项目上修复，相关commit讨论记录如下：
https://github.com/Ameba-AIoT/ameba-arduino-pro2/pull/259

本repo中的WiFiClient.cpp文件，即是将上述修复合入到BW16
Arduino源码中的结果。编译前，使用本文件覆盖BW16源码中对应文件即可。

BW16 arduino工程源码：
https://github.com/Ameba-AIoT/ameba-arduino-d

BW16库文档：
https://ameba-doc-arduino-sdk.readthedocs-hosted.com/en/latest/ameba_d/bw16-typeb/index.html

