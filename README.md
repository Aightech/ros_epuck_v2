# ros_epuck_v2
This project is a ros node aiming to control e-puck2 via bluetooth.

## Requirement
```bash
sudo apt-get install libbluetooth-dev bluez
```
## Bluetooth Connection
---
**Warning**

By default, the e-puck2 is not visible when you search for it in the Bluetooth utility of your computer.
To make it visible, it is necessary to hold the USER button (also labeled "esp32" on the electronic board) while turning on the robot with the ON/OFF button. (cf: *Connecting to the Bluetooth* http://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development)

---

### Scan bluetooth devices
you can find the mac adress of your epuck2 with :
```bash
sudo hcitool scan
Scanning ...
	30:AE:A4:03:77:8E	e-puck2_04066
```
keep this mac address for the following command.

You can get the info of the e-puck2:
```bash
sudo hcitool info 30:AE:A4:03:77:8E
Requesting information ...
	BD Address:  30:AE:A4:03:77:8E
	OUI Company: Espressif Inc. (30-AE-A4)
	Device Name: e-puck2_04066
	LMP Version: 4.2 (0x8) LMP Subversion: 0x30e
	Manufacturer: RivieraWaves S.A.S (96)
	Features page 0: 0xbf 0xee 0xcd 0xfe 0xdb 0xff 0x7b 0x87
		<3-slot packets> <5-slot packets> <encryption> <slot offset> 
		<timing accuracy> <role switch> <sniff mode> <RSSI> 
		<channel quality> <SCO link> <HV3 packets> <u-law log> 
		<A-law log> <CVSD> <power control> <transparent SCO> 
		<broadcast encrypt> <EDR ACL 2 Mbps> <EDR ACL 3 Mbps> 
		<enhanced iscan> <interlaced iscan> <interlaced pscan> 
		<inquiry with RSSI> <extended SCO> <EV4 packets> <EV5 packets> 
		<AFH cap. slave> <AFH class. slave> <LE support> 
		<3-slot EDR ACL> <5-slot EDR ACL> <sniff subrating> 
		<pause encryption> <AFH cap. master> <AFH class. master> 
		<EDR eSCO 2 Mbps> <EDR eSCO 3 Mbps> <3-slot EDR eSCO> 
		<extended inquiry> <LE and BR/EDR> <simple pairing> 
		<encapsulated PDU> <err. data report> <non-flush flag> <LSTO> 
		<inquiry TX power> <EPC> <extended features> 
	Features page 1: 0x03 0x00 0x00 0x00 0x00 0x00 0x00 0x00
	Features page 2: 0x5f 0x03 0x00 0x00 0x00 0x00 0x00 0x00

```

```bash 
sudo rfcomm bind /dev/rfcomm0 30:AE:A4:03:77:8E 2
```
Now you can use /dev/rfcomm0 to connect to the e-puck2. 

---
**Test**
```bash
  sudo apt install screen
```
```bash
  screen /dev/rfcomm0
```
Enter the letter 'A' then [ret] several times to send the ascii character 'A' to the epuck. It should return the values 
```bash
a,2051,2015,2794
a,2054,2016,2790
a,2053,2018,2791
...
```

refer to http://www.gctronic.com/doc/index.php/Advanced_sercom_protocol to see other ascii commands.

If the screen command gives you :
```bash
[screen is terminating]
```
you might not have the right permission. To set the right permission you can do :
```bash
sudo chmod a+w /dev/rfcomm0 
```




---
