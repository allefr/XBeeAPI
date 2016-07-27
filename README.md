# XBeeAPI
python implementation for interfacing a computer/raspberry Pi/beagleBone to any XBee module setup to use API mode (either with or without escaping sequence).

All message types are coded as classes, children of parent class XBee_msg which includes shared methods.


## Features
This implementation is including all the main features described in the DigiMesh API:
- [x] setLocalRegister()
- [x] getLocalRegister()
- [x] setRemoteRegister()
- [x] getRemoteRegister()
- [x] sendDataToRemote()
- [x] readSerial()

also including some more specific and usually not implemented features:
- [x] networkDiscover()
- [x] findNeighbors()
- [x] traceRoute()
- [x] linkQualityTest()


### create XBee object
Before using any other methods, a XBee API object must be created.

It is possible to configure some XBee registers by passing the following parameters:
- ```port```: serial port (default: a method is called to find a possible XBee serial instance from the
system);
- ```baud```: baud rate of the serial communication (default: ```9600```);
- ```ID```: Network ID (default: ```0x7FFF```);
- ```AP```: API Mode1 (default: ```0x02```, API mode with escaping sequence enabled);
- ```AO```: API Output Format (default: ```0x00```, standard data frame);
- ```NO```: Network Discovery Options (default: ```0x04```, appended RSSI to network/neighbor discovery
replies).
If any of these parameters is not provided, the default value is used.
Note that the default value may be different from the factory reset. Note also that when using the XBee for the first time, it is mandatory to first set it as AP = 2 otherwise no communication is possible by using the provided interface.

A possible, basic example can be:
```
from XBee_API import XBee_API
XB = XBee_API()
```

If the XBee module was found in the list of serial ports available, the XBee is initialized. If no XBee
Module can be found, an error is raised.

### setLocalRegister()
Method used for setting a register in the local XBee Device.
This method requires the following parameters:
- ```command```: local AT command as 2 ASCII string;
- ```value```: value to assign to the AT command;
- ```frame_ID``` (default: ```0x52```): can be set differently if mission specific.

A possible example can be:
```
XB.setLocalRegister(’AP’, 0x02)
```

Note that the register value can be expressed also as hex string (```’02’```) or as bytearray (```bytearray([0x02])```).
The method will generate and send the created DigiMesh frame to the serial communication.

It outputs the created XBee message object, which - for instance - can be then printed in the form:
```
2016-07-04 12:43:22.871 OUT (addr: local ) Set registry ’AP’ to ’02’
```

### getLocalRegister()
Method used for getting a register in the local XBee Device.
This method requires the following parameters:
- ```command```: local AT command as 2 ASCII string;
- ```frame_ID``` (default: ```0x52```): can be set differently if mission specific.

A possible example can be:
```
XB.getLocalRegister(’ID’)
```

The method will generate and send the created DigiMesh frame to the serial communication.

It outputs the created XBee message object, which - for instance - can be then printed in the form:
```
2016-07-04 12:43:23.428 OUT (addr: local ) Get ’ID’ registry
```

### setRemoteRegister()
Method used for setting a register to a remote XBee Device.
This method requires the following parameters:
- ```destH```: high address of the remote device as a hex string;
- ```destL```: low address of the remote device as a hex string;
- ```command```: local AT command as 2 ASCII string;
- ```value```: value to assign to the AT command;
- ```frame_ID``` (default: ```0x52```): can be set differently if mission specific.

A possible example can be:
```
XB.setRemoteRegister(’0013a200’, ’40e44b94’, ’AP’, 0x02)
```

Note that the register value can be expressed also as string (```’02’```) or as bytearray (```bytearray([0x02])```).
The method will generate and send the created DigiMesh frame to the serial communication.

It outputs the created XBee message object, which - for instance - can be then printed in the form:
```
2016-07-04 12:43:22.871 OUT (addr: 40e44b94) Set registry ’AP’ to ’02’
```


### getRemoteRegister()
Method used for getting a register to a remote XBee Device.
This method requires the following parameters:
- ```destH```: high address of the remote device as a hex string;
- ```destL```: low address of the remote device as a hex string;
- ```command```: local AT command as 2 ASCII string;
- ```frame_ID``` (default: ```0x01```): can be set differently if mission specific.

A possible example can be:
```
XB.getRemoteRegister(’0013a200’, ’40e44b94’, ’ID’)
```

The method will generate and send the created DigiMesh frame to the serial communication.

It outputs the created XBee message object, which - for instance - can be then printed in the form:
```
2016-07-04 12:43:23.428 OUT (addr: 40e44b94) Get ’ID’ registry
```

###  sendDataToRemote()
Method used for sending data to a remote XBee Device.
This method requires the following parameters:
- ```destH```: high address of the remote device as a hex string;
- ```destL```: low address of the remote device as a hex string;
- ```data```: data to send, formatted as bytearray;
- ```frame_ID``` (default: ```0x01```): can be set differently if mission specific;
- ```option``` (default: ```0x00```): set to 0x08 for Route Tracing;
- ```reserved``` (default: ```0xFFFE```): set to ```0xFFFF``` for Route Tracing.

A possible example can be:
```
data = bytearray([0x01, 0x02, 0x03])
XB.sendDataToRemote(’0013a200’, ’40e44b94’, data)
```
Note that the data must be format as bytearray.

The method will generate and send the created DigiMesh frame to the serial communication.

It outputs the created XBee message object, which - for instance - can be then printed in the form:
```
2016-07-04 12:44:23.563 OUT (addr: 40e44b94) data: hex’010203’
```

### readSerial()
Method used for reading data from the serial communication with the local XBee. 
Note that this function only reads until data are inside the serial buffer. 
The readSerial() method should be called iteratively at fast rates.

This method requires no parameters.

The data read from the serial buffer is continuously appended in a local buffer. 
The local buffer is split in one or several frames by using the DigiMesh start delimiter character 0x7E.
A XBee msg object is created given each frame by using the ```frame_type```; if the validation procedure is successful, the frame is removed from the local buffer and the created object is appended to a local
list containing all the received messages.

The received-message list is then returned as output of the readSerial() method and will be emptied at the next call of readSerial() method.

The user can extract each object using:
```
XBmsgList = XB.readSerial()
while XBmsgList
XBmsg = XBmsgList.pop(0)
```
It is now possible to recognize each message by ```XBmsg.frame_type```. For instance, if ```frame_type``` = ```0x90```, it is possible to access the RF data by using ```XBmsg.data``` (in bytearray).


## XBee Testing Methods
The following methods are provided by the DigiMesh API [RD1], with recommendation to use them for test purposes only. 
The reason is that they could potentially fail, giving misleading values if not properly used or taking considerable amount of time for real-time implementations.

### networkDiscover()
The Network Discovery feature allows to query each XBee module sharing the same Network ID and within RF range of at least one other XBee, no matter how many hops are needed to reach every individual in the network.
The DigiMesh API provides a register for defining the maximum time for waiting for a reply to the Network Discovery command; default value is 13 seconds. It is advised to higher up this value in case of large network.
Because of this reason, this feature cannot be used for real-time implementations.

The Network Discovery command could be potentially sent to a remote XBee module, but the reply would be the same, so it was implemented for addressing the local XBee only.

No input parameters are needed and no output parameter.

By sending the following command:
```
XB.networkDiscover()
```
several responses (one from each device) are received in the form:
```
2016-07-07 13:33:20.555 IN (addr: local ) Registry ’ND’ = ’FFFE0013A20040E44BA92000FFFE0100C105101E38’; [OK]
```

In this example case, the received data can be interpreted in the following way:
- ```reserved```: ```’FFFE’```
- high address: ```’0013A200’```
- low address: ```’40E44BA9’```
- node identifier: ```’2000’```
- parent network address: ```’FFFE’```
- device type: ```’01’``` (```’00’```: coordinator, ```’01’```: router, ```’02’```: end-point)
- reserved: ```’00’```
- profile ID: ```’C105’```
- manufacturer ID: ```’101E’```
- RSSI of last hop: ```’38’``` = −0x38dBm = −56dBm. Note that this additional value is a consequence
of having set ```’NO’``` = ```0x04```.


### findNeighbors()
Find Neighbors feature is very similar to the Network Discover feature as it produce the same kind of reply. The difference is the range of devices it can reach; only RF reachable devices will reply to
the Find Neighbors command.

It might be of interest therefore to know the neighbors of both local and remote XBee modules, as a remote device can have different neighbors then the locale.
```findNeighbors()``` method is therefore requiring the high and slow addresses of the module to enquire.
```’LOCAL’``` can be set for either high or low address to find neighbors of the local XBee module.

A possible example can be:
```
XB.findNeighbors(’0013a200’, ’40e44ba9’)
```
or
```
XB.findNeighbors(’’, ’LOCAL’)
```

Any response would be in the form:
```
2016-07-08 18:25:31.140 IN (addr: local ) Registry ’FN’ = ’FFFE0013A20040D4B3E72000FFFE0100C105101E48’; [OK]
```
which can be interpreted in the same way as the Network Discovery response.


### traceRoute()
Trace Routing is a feature recommended for testing setups only capable of providing route information from the local XBee module to a remote one, maybe not within RF range.

The command requests as input the high and low addresses of the module to route tracing.
It is not allowed to attempt Route Tracing for the local XBee. It is also not allowed to broadcast a
route tracing command.

After sending the following command:
```
XB.traceRoute()
```
one or several responses (one from each hop) are received in the form:
```
2016-07-08 18:39:06.104 IN (addr: 40e44b40) Route Info ’40e44b40’ to ’40d8789e’; receiver: 40d4b3e7
```

In this example, the route tracing request was from module ```’40e44b40’``` (local) to ```’40d8789e’```. 
The communication was sent from ```’40e44b40’``` (local) to the next hop, in this case ```’40d4b3e7’```.

The next response would look like:
```
2016-07-08 18:39:06.208 IN (addr: 40d4b3e7) Route Info ’40e44b40’ to ’40d8789e’; receiver: 40d8789e
```
meaning that, for the same route tracing command (from ```’40e44b40’``` (local) to ```’40d8789e’```), the communication is now between node ```’40d4b3e7’``` and the final node ```’40d8789e’```.
In this example there was only one hop between the local XBee module and the target ```’40d8789e’```, thus only 2 responses were received.


### linkQualityTest()
The DigiMesh API [RD1] features the capability of internally (between the Xbee modules) testing the quality link of 2 devices in RF range between themselves.

The test allows to set a number of bytes to exchange between the 2 devices and a number of iterations for repeating the exchange.
If the transfer is successful a report is given back providing quality link data.

The Link can be performed between any device to any other device, as long as the two are within RF range between each other.

Information about the source (high and low) address and destination (high and low) address is to be passed to the method.
It is also possible to define the number of bytes to exchange and the iterations. If these are not provided, the default values of 32 bytes and 200 iterations are used respectively.

A possible example can be:
```
XB.linkQualityTest(XB.params[’SH’], XB.params[’SL’], ’0013a200’, ’40e44ba9’)
```

The response is in the form:
```
2016-07-08 18:15:51.811 IN (addr: local ) explicit transmit data: hex’0013A20040D4B3E7002000C800C80017000A494C49’; [status: 0]
```

where the data can be interpreted as:
- high address: ```’0013A200’```
- low address: ```’40D4B3E7’```
- payload size: ```’0020’``` (= 32 bytes)
- iterations: ```’00C8’``` (= 200)
- number of successful iterations: ```’00C8’```
- number of retries: ```’0017’```
- result: ```’00’``` (```’00’```: success, ```’03’```: invalid parameter)
- maximum number of MAC retries: ```’0A’```
- maximum RSSI: ```’49’``` (= −73dBm)
- minimum RSSI: ```’4C’``` (= −76dBm)
- average RSSI: ```’49’``` (= −73dBm)


## Threading
There is no threading implemented so far as it was not needed for the applications where this code was used.
A possible future development may include the implementation of threads.

Note however that it is needed a was to cyclically call the ```readSerial()``` method.

So far this code was successfully implemented when:
- using graphical interface;
- creating a ROS package, calling ```readSerial()``` from a ROS node.

## Contribution
This code was based on a different implementation done by @bzoss