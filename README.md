# hfp_hf_my  
Bluetooth HF(Hand Free) Profile Sample program.  

ESP-EYE の Microphone を、PC で聞きたくてつくてみました。  
Bluetooth Classic hfp client プログラムです。  
Sink Speaker -- I2S には、出力していません。受け捨て状態です。  
Source Microphone  -- ESP-EYE の Mic から、I2S で取り込んでいます。  

#### 開発環境  
Ubuntu Mate 24.04  
Vscode and ESP-IDF 拡張機能  
eps-idf v5.5  
Original Source code  
Base  
[@espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/classic_bt/hfp_hf](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/classic_bt/hfp_hf)  
Reffer  
[@espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/classic_bt/hfp_ag](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/classic_bt/hfp_ag)
