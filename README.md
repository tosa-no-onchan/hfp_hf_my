# hfp_hf_my  
Bluetooth HF(Hand Free) Profile Sample program.  
  
##### ESP-EYE Microphone bluetooth hfp client (source Microphone and sink Speaker) program.  

ESP-EYE の Microphone を、PC で聞きたくてつくてみました。  
Bluetooth Classic hfp client プログラムです。  
Sink Speaker -- I2S には、出力していません。受け捨て状態です。  
Source Microphone  -- ESP-EYE の Mic から、I2S で取り込んでいます。  

#### 使い方  
1. Ubuntu 24.04 PC に、Bluetooth USB ドングルをさして、  
ESP_HFP_HF ペアリングを許可して、電話帳を拒否する。  
2. サウンド -> 入力 を設定する。  
   ◎ ESP_HFP_HF  
3. PC ターミナルから、
   $ python mic2Speaker.py  

#### 開発環境  
Ubuntu Mate 24.04  
Vscode and ESP-IDF 拡張機能  
eps-idf v5.5  
Original Source code  
Base hf sink 部分    
[@espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/classic_bt/hfp_hf](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/classic_bt/hfp_hf)  
hf source 部分を、一部ポーティングしました。    
[@espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/classic_bt/hfp_ag](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/classic_bt/hfp_ag)

#### 参照  
[ESP-EYE マイク を bluetooth で、PC につなげて聞きたい。](https://www.netosa.com/blog/2025/08/esp-eye-bluetooth-pc.html)  
