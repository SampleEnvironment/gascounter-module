@mainpage


@section Compileconfog Compiling for LAN or Xbee

There are two different hardware versions of the Gascounter Module, one uses a wired interface to the Server (LAN), and the other a wireless connection (Xbee). The source code needs to be configured correctly before compiling the code and flashing it onto the device. 

@subsection LANconf LAN configuration

 When the LAN variant is used, data will take the following path: 
 &micro;c → USART → XPort → LAN → Server-COMport → Database

To configure the Source code correctly:

1.  #USE_LAN has to be defined, and #USE_XBEE has to be undefined in Gascounter_main.h
2. The Gascounter Module specific IP-Address has to be entered in #IP_1_octet to #IP_4_octet (gc_xbee_utilities.c) 

@subsection Xbeeconf Xbee configuration

When the Xbee variant is used, data will take the following path:
 µc → USART → Xbee-Module → Coordinator → Database
 
 To configure the Source code correctl #USE_LAN has to be undefined, and #USE_XBEE has to be defined in Gascounter_main.h


<hr>


@section Statechart State Diagram

The state diagram represents the general workings of a Gascounter module. The diagram does not completly cover all the details, but is definitely good enought to understand the main routines. All States in which the Gascounter is in Online mode (#ex_mode_online) are colored in <span style="color:#00cc00"><b>green</b></span>. When it is in Offline mode (#ex_mode_offline) the states are colored <span style="color:red"><b>red</b></span>. Some states are also colored in <span style="color:gray"><b>gray</b></span> to represent the possible transitions form Offline to Online mode or vice versa. The unrecoverable Errorstate colored in <span style="color:blue"><b>blue</b></span> can only be reached if there was any error during #init, #xbee_send_login_msg or #Set_Options.
<img src="Gaszaehler_stc.jpg" alt="Gascounter State chart" width="1300" >
\image latex Gaszaehler_stc.jpg "Gascounter State chart" width=15cm

<hr>

@section Display The User Interface (LCD Screen)

The LCD screen of the Gascounter modules has the ability to display six lines of text with twelve characters each. This means that the device´s capability to display a lot of status information at the same time is limited. A typical state of the Display during normal operation can be seen in the figure below. 

<img src="LCD_Template.jpg" alt="LCD scheme" width="1300" >
\image latex LCD_Template.jpg "LCD scheme" width=15cm

The first three lines are reserved for displaying the current volume reading.

The fourth and fifth line are for displaying Temerature and Pressure measured by the BMP Sensor connected to the device (If options#p_Compensation_enable or options#T_Compensation_enable is set to zero the String "No Pcomp" and/or "No Tcomp" are displayed). If Temperature or Pressure compensation is enabled and there is an error with the BMP sensor the String "TEMP ERR" and/or "PRESS ERR" are printed respectively. Additionally a more detailed Error message is printed every five seconds on the whole screen.  

System time, activity indicator and a six character long Status-Field are displayed in the last line. The activity indicator changes its apperance every #Measure_Interval (5s). This way one can quickly discern if the device is still running properly or if it stopped working. All possible statusmessages shown in the Status-Field and a short explanation can bes found in the table below. One of the first three entrys is displayed while the device is idleing. All the other entrys are triggered by send/receive events. 

Status-Field String| Explanation
---|---
"Online"|Device is connected to the Server 
"NoServ"|No connection to the Server, but the Xbee is associated to a coordinator
"NoNetw"|No connection to the Network
" Op96 "|Setting Options received from server
" Op93 "|Sending "options accepted" Message to server 
"Ping.."|Ping sent to server waiting for Pong
"PingOk"|Pong received
"NoPong"|#COM_TIMEOUT_TIME exceeded and no Pong received
"sendMs"|Mostly measurement Messages (#CMD_send_data_91)
"sendOk"|ACK corresponding to message received
"No Ack"|#COM_TIMEOUT_TIME exceeded and no ACK received
"reset "|connection reset (reassociation with coordinator started)
"AD err"|An error occurred while aquiring Network adresses

<hr>


@section messageformat Xbee Message Format

@subsection txframe Xbee TX Frame

More info can be found in <a href="XBEE Datasheet.pdf" target="_blank"><b>Xbee Datasheet</b></a>

 Bytenumber| Description  
--- | --- 
 [0] | Start delimiter
[1..2] |  frame length (excludes Start delimiter, Length and checksum)
 [3] | API Identifier for 64-bit TX Request
 [4] | Frame ID, if set to '0' response frame is disabled
 [5..12] | Destination 64bit-Address MSB first, LSB last 
 [13] |  Disable acknowledgment 
  [14]| Database command type 
 [15..n] | RF DATA (content)
  [n+1] | checksum



@subsection MeasSend Message Format for sending Measurements

The Measurement message format has a length of 23 bytes (#MEASUREMENT_MESSAGE_LENGTH). Measurements are usually sent with the #CMD_send_data_91 command, but also as an answer to the #CMD_received_send_data_97 command (#CMD_send_response_send_data_94). All measurement Messages are Acked by the Server. 

Byte Number | content 
--- | ---
[0] | Seconds (#timeType)
[1] | Miuntes (#timeType)
[2] | Hours (#timeType)
[3] | Date (#timeType)
[4] | Month (#timeType)
[5] | Year (#timeType)
[6..9] | Value in liters (#optType)
[10..13] | Volume in liters (#optType)
[14..17] | Corrected Volume in liters (#optType)
[18..19] | #Temperature_value 
[20..21] | #Pressure_value 
[22] | status#byte_91 (#statusType)


@subsection OptsRecSend Message Format for receiving and sending Options

Options are received with the #CMD_received_set_options_96 command. The options meassage format has a length of 39 bytes (#NUMBER_LOGIN_BYTES). A shorter version is packed when sending the current options upon receiving the #CMD_received_send_options_98 command (The first six byte containing time and date are omitted).

Byte Number | content 
--- | ---
[0] | Seconds (#timeType)
[1] | Miuntes (#timeType)
[2] | Hours (#timeType)
[3] | Date (#timeType)
[4] | Month (#timeType)
[5] | Year (#timeType)
[6..9] | offsetValue in liters (#optType)
[10..13] | offsetVolume in liters (#optType)
[14..17] | offsetCorrVolume in liters (#optType)
[18..19] | t_transmission_min (#optType)
[20..21] | t_transmission_max  (#optType)
[22..23] | delta_V (#optType)
[24..25] | delta_p (#optType)
[26..27] | step_Volume (#optType)
[28..29] | offset_pressure (#optType)
[30..31] | span_pressure (#optType)
[32] | T_Compensation_enable (#optType)
[33..34] | Temperature_norm (#optType)
[35] | p_Compensation_enable (#optType)
[36..37] | Pressure_norm (#optType)
[38] | status#byte_96 (when receiving), status#byte_92 (when sending) (#statusType)

@subsection otherFormats Other Message Formats

Other message types when receiving or sending only contain the corresponding status#byte_*  at index [0]. 


<hr>


@section Status Device Status and Errors

All the Status and Error handling is done in #status and #ex_mode, and can be further subdivided in internal and external status and error handling.

@subsection intern Internal Status and Error handling
The internal state (#ex_mode) is governed by the status byte in status#device. Each of the first six bits in status#device corresponds to a different kind of <b>error</b>.

<b>status#device bit assignment:</b>

0|1|2|3|4|5
--- | --- | --- | --- | --- | --- |
NETWORK_ERROR | NO_REPLY_ERROR | OPTIONS_ERROR | TEMPPRESS_ERROR | TIMER_ERROR | INIT_OFFLINE_ERROR 


+ <b>NETWORK_ERROR:</b>
Network connection to the server was lost and the Gascounter module switches to #ex_mode_offline. The Error can be resolved by trying a #xbee_reconnect or a successful #ping_server.

+ <b>NO_REPLY_ERROR:</b>
It is set if a Ping Meassage (#CMD_send_Ping_95) was not answered by the Server. It can be resolved by a succesful Ping. (deprecated: same functionality as NETWORK_ERROR)

+ <b>OPTIONS_ERROR:</b>
Set if faulty Options were received. If it occurs on startup, the device goes into the unrecoverable error state(#ex_mode_error). Otherwise old options are kept.

+ <b>TEMPPRESS_ERROR:</b>
It indicates an issue with the BMP085 Sensor. It can only be set if either Temperature- or Pressure-Compensation are enabled. If it is set, the Gascounter will use the last valid Temerature and Prassure Values to calculate the corrected Volume. If it occurs on startup, the device will enter the unrecoverable error state(#ex_mode_error).

+ <b>TIMER_ERROR:</b>
It indicates that either the DS3231M realtime clock is disconnected or faulty. Time and date bytes are all set to zero. If it occurs on startup, the device will enter the unrecoverable error state(#ex_mode_error). Otherwise it will continue to operate normaly. 

+ <b>INIT_OFFLINE_ERROR:</b>
It will only be set if no network connection could be established on startup, and leads always to the unrecoverable errorstate (#ex_mode_error)



@subsection extern External Status and Error communication

The communication of device status and errors is always specific to the kind of CMD_send_* or CMD_received_* messages. With every CMD message type the corresponding status_byte is sent.



CMD| status#byte
---|---
#CMD_send_registration_90 | The status#byte_90 is currently always set to zero.
#CMD_send_data_91 | status#byte_91 is part of every measurement data transmission and communicates the internal state of the device (status#device) to the server. Assignment of the first four bits are #status_bit_option_err_91, #status_bit_Temp_Press_Err_91, #status_bit_volume_too_big_91 and #status_bit_DS3231M_err_91.
#CMD_send_options_92 | The status#byte_92 is currently always set to zero.
#CMD_send_response_options_set_93 | status#byte_93 is sent as an answer to receiving options from the server. If the 0th bit (#status_bit_success_setting_options_93) is set to zero the Options were accepted and if it is set to one, the options were faulty. 
 #CMD_send_Ping_95 | The status#byte_92 is currently always set to zero.
  #CMD_received_set_options_96 | When receiving options, the 0th bit in statu#byte_96 (it is transmitted with the options) indicates if volume offsets should be kept (set to zero), or set to the values transmitted in the #CMD_received_set_options_96 message. 
  
  
 

 


