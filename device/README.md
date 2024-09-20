Currently building a Nursing Home Emergency Detection System using the ESP32S3 MCU.

System will have **devices** for each resident to wear on their person.\
There will also be **stations** sparsely placed around the Retirement Home.

The devices will monitor users' vital signs (temperature, heartrate), and transmit this information periodically
to the Nurse's Office (a local server on a desktop computer).\
If there is ever a dangerous reading on a user's device, the device will immediately transmit the information
to the nearest station. The station will then transmit the data to the Nurse's Office in real time.

The devices will implement BLE as broadcasters to transmit their readings to the stations.\
The stations will use WiFi to be able to transmit the information they receive to the Nurses' Office (wherever that may be). 

I chose this structure as the system is intended to work in large Retirement Communities, housing many residents,
and within large buildings. The devices themselves will run on battery, so they should be low-powered. This discouraged
me from having each device be connected to WiFi. Instead, I opted for Bluetooth Low Energy to implement the
wireless communication from the devices.\
However, BLE should not be relied upon to transmit data anywhere within a large building, as the signal would
almost certainly be obstructed by walls, floors, distance, etc. This led me to introduce another ESP32 to serve as
a station to relay the information from devices. These stations would be connected to a consistent power supply 
and thus are less restrained on power consumption. Therefore, WiFi will not be a problem for the stations.\
WiFi also allows devices to communicate anywhere within a WiFi network regardless of the obstructions that swayed
me from solely using BLE. The transmission of data to the Nurses' Station needs to be reliable and fast, so
WiFi seems like the right choice to do this considering that an emergency can happen anywhere within the Home.
