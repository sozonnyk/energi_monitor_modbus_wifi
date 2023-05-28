#include <secrets.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ModbusMaster.h>
#include <ArduinoHA.h>
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#define WIFI_WAIT_TIME_MS 10000
#define TZ_DEF "AEST-10AEDT,M10.1.0,M4.1.0/3"
#define NTP_SERVER "time.google.com"

#define REFERENCE_RSSI 60 //69 at 0dBm
#define BT_POWER ESP_PWR_LVL_P9 //ESP_PWR_LVL_N0 for 0 dBm
#define BT_SCAN_SEC 5
#define BT_DISTANCE_COEFF 2

#define RED "F7:3C:02:77:48:F9"
#define YELLOW "C3:AD:CA:7D:C2:13"
#define GREEN "FA:26:C1:CD:98:1F"

#define MAIN_METER_ID 44
#define SOLAR_METER_ID 12

BLEAddress red = BLEAddress(RED);
BLEAddress yellow = BLEAddress(YELLOW);
BLEAddress green = BLEAddress(GREEN);

ModbusMaster mainMeterNode;
ModbusMaster solarMeterNode;

AsyncWebServer server(80);
WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device, 23);

int previousMinute = -1;

HASensorNumber voltageASensor("voltage_a", HASensorNumber::PrecisionP2);
HASensorNumber voltageBSensor("voltage_b", HASensorNumber::PrecisionP2);
HASensorNumber voltageCSensor("voltage_c", HASensorNumber::PrecisionP2);
HASensorNumber voltageSSensor("voltage_s", HASensorNumber::PrecisionP2);

HASensorNumber currentASensor("current_a", HASensorNumber::PrecisionP3);
HASensorNumber currentBSensor("current_b", HASensorNumber::PrecisionP3);
HASensorNumber currentCSensor("current_c", HASensorNumber::PrecisionP3);
HASensorNumber currentSSensor("current_s", HASensorNumber::PrecisionP3);

HASensorNumber fullPowerSensor("full_power", HASensorNumber::PrecisionP1);
HASensorNumber exportPowerSensor("export_power", HASensorNumber::PrecisionP1);

HASensorNumber powerASensor("power_a", HASensorNumber::PrecisionP1);
HASensorNumber powerBSensor("power_b", HASensorNumber::PrecisionP1);
HASensorNumber powerCSensor("power_c", HASensorNumber::PrecisionP1);
HASensorNumber powerSSensor("power_s", HASensorNumber::PrecisionP1);

HASensorNumber freqSensor("freq", HASensorNumber::PrecisionP1);
HASensorNumber totalEnergySensor("total_power", HASensorNumber::PrecisionP1);
HASensorNumber totalSolarEnergySensor("total_powe_Sr", HASensorNumber::PrecisionP1);
HASensorNumber rssi("rssi", HASensorNumber::PrecisionP0);

HASensorNumber distanceRedSensor("distance_red", HASensorNumber::PrecisionP2);
HASensorNumber distanceYellowSensor("distance_yellow", HASensorNumber::PrecisionP2);
HASensorNumber distanceGreenSensor("distance_green", HASensorNumber::PrecisionP2);

float reform_uint16_2_float32(uint16_t u1, uint16_t u2) {
	uint32_t num = ((uint32_t) u1 & 0xFFFF) << 16 | ((uint32_t) u2 & 0xFFFF);
	float numf;
	memcpy(&numf, &num, 4);
	return numf;
}

float getRTU(ModbusMaster node, uint16_t m_startAddress) {
	uint8_t m_length = 2;
	uint16_t result;

	result = node.readInputRegisters(m_startAddress, m_length);
	if (result == node.ku8MBSuccess) {
		return reform_uint16_2_float32(node.getResponseBuffer(0),
				node.getResponseBuffer(1));
	} else {
		return -1;
	}
}

struct distance {
	float red;
	float yellow;
	float green;
};

distance getDistance() {
	Serial.println("BT scan");
	BLEDevice::setPower(BT_POWER, ESP_BLE_PWR_TYPE_SCAN);
	BLEScan *scan = BLEDevice::getScan();
	scan->setActiveScan(true);
	BLEScanResults results = scan->start(BT_SCAN_SEC);
	Serial.println("Scan done");

	distance result;
	result.green = -1;
	result.red = -1;
	result.yellow = -1;

	for (int i = 0; i < results.getCount(); i++) {
		BLEAdvertisedDevice device = results.getDevice(i);
		BLEAddress address = device.getAddress();
		float distance = pow(10,
				((-(float) REFERENCE_RSSI - device.getRSSI()) / ((float) 10 * BT_DISTANCE_COEFF)));

		Serial.print(address.toString().c_str());
		Serial.print(" ");
		Serial.println(distance);

		if (address.equals(red)) result.red = distance;
		if (address.equals(yellow)) result.yellow = distance;
		if (address.equals(green)) result.green = distance;
	}

	return result;
}

void initTime() {
	configTime(0, 0, NTP_SERVER);
	setenv("TZ", TZ_DEF, 1);
	tzset();
}

void initOta() {
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send(200, "text/plain", "Power station");
	});

	AsyncElegantOTA.begin(&server);
	server.begin();
}

void initWifi(){
	WiFi.setHostname("powerstation");
	WiFi.mode(WIFI_STA);
	WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
	WiFi.begin(WIFI_SSID, WIFI_PASSWD);
	while (WiFi.waitForConnectResult() != WL_CONNECTED) {
		Serial.println("Connection Failed! Rebooting...");
		delay(5000);
		ESP.restart();
	}
	Serial.print("IP: ");
	Serial.println(WiFi.localIP());
}


void setup() {
	Serial.begin(115200);
	Serial2.begin(9600, SERIAL_8E1);
	mainMeterNode.begin(MAIN_METER_ID, Serial2);
	solarMeterNode.begin(SOLAR_METER_ID, Serial2);

	BLEDevice::init("");

	initWifi();
	initTime();
	initOta();

	byte mac[6];
	WiFi.macAddress(mac);
	device.setUniqueId(mac, sizeof(mac));
	device.setName("Power Meter");
	device.setSoftwareVersion("1.0.0");
	device.setManufacturer("Andrew");
	device.setModel("Power Meter");

	voltageASensor.setIcon("mdi:power-socket-au");
	voltageASensor.setName("Voltage A");
	voltageASensor.setUnitOfMeasurement("V");

	voltageBSensor.setIcon("mdi:power-socket-au");
	voltageBSensor.setName("Voltage B");
	voltageBSensor.setUnitOfMeasurement("V");

	voltageCSensor.setIcon("mdi:power-socket-au");
	voltageCSensor.setName("Voltage C");
	voltageCSensor.setUnitOfMeasurement("V");

	voltageSSensor.setIcon("mdi:power-socket-au");
	voltageSSensor.setName("Voltage Solar");
	voltageSSensor.setUnitOfMeasurement("V");

	currentASensor.setIcon("mdi:lightning-bolt");
	currentASensor.setName("Current A");
	currentASensor.setUnitOfMeasurement("A");

	currentBSensor.setIcon("mdi:lightning-bolt");
	currentBSensor.setName("Current B");
	currentBSensor.setUnitOfMeasurement("A");

	currentCSensor.setIcon("mdi:lightning-bolt");
	currentCSensor.setName("Current C");
	currentCSensor.setUnitOfMeasurement("A");

	currentSSensor.setIcon("mdi:lightning-bolt");
	currentSSensor.setName("Current Solar");
	currentSSensor.setUnitOfMeasurement("A");

	fullPowerSensor.setIcon("mdi:meter-electric-outline");
	fullPowerSensor.setName("Power");
	fullPowerSensor.setUnitOfMeasurement("W");

	exportPowerSensor.setIcon("mdi:meter-electric-outline");
	exportPowerSensor.setName("Power Export");
	exportPowerSensor.setUnitOfMeasurement("W");

	powerASensor.setIcon("mdi:transmission-tower");
	powerASensor.setName("Power A");
	powerASensor.setUnitOfMeasurement("W");

	powerBSensor.setIcon("mdi:transmission-tower");
	powerBSensor.setName("Power B");
	powerBSensor.setUnitOfMeasurement("W");

	powerCSensor.setIcon("mdi:transmission-tower");
	powerCSensor.setName("Power C");
	powerCSensor.setUnitOfMeasurement("W");

	powerSSensor.setIcon("mdi:transmission-tower");
	powerSSensor.setName("Power Solar");
	powerSSensor.setUnitOfMeasurement("W");

	freqSensor.setIcon("mdi:sine-wave");
	freqSensor.setName("Frequency");
	freqSensor.setUnitOfMeasurement("Hz");

	totalEnergySensor.setIcon("mdi:meter-electric-outline");
	totalEnergySensor.setName("Energy");
	totalEnergySensor.setUnitOfMeasurement("kWh");
	totalEnergySensor.setDeviceClass("energy");
	totalEnergySensor.setStateClass("total_increasing");

	totalSolarEnergySensor.setIcon("mdi:meter-electric-outline");
	totalSolarEnergySensor.setName("Energy Solar");
	totalSolarEnergySensor.setUnitOfMeasurement("kWh");
	totalSolarEnergySensor.setDeviceClass("energy");
	totalSolarEnergySensor.setStateClass("total_increasing");

	rssi.setIcon("mdi:wifi");
	rssi.setName("RSSI");
	rssi.setUnitOfMeasurement("RSSI");

	distanceRedSensor.setIcon("mdi:trash-can");
	distanceRedSensor.setName("Distance Red Bin");
	distanceRedSensor.setUnitOfMeasurement("m");

	distanceGreenSensor.setIcon("mdi:trash-can");
	distanceGreenSensor.setName("Distance Green Bin");
	distanceGreenSensor.setUnitOfMeasurement("m");

	distanceYellowSensor.setIcon("mdi:trash-can");
	distanceYellowSensor.setName("Distance Yellow Bin");
	distanceYellowSensor.setUnitOfMeasurement("m");

	mqtt.begin(MQTT_HOST, MQTT_USER, MQTT_PASSWD);
}

void loop() {
	struct tm time;
	getLocalTime(&time);

	if (time.tm_min != previousMinute) {
		previousMinute = time.tm_min;

		Serial.println(&time, "%A, %B %d %Y %H:%M:%S");

		if (WiFi.status() != WL_CONNECTED) {
			Serial.println("WiFi lost");
			WiFi.disconnect();
			WiFi.reconnect();
			if (WiFi.waitForConnectResult(WIFI_WAIT_TIME_MS) != WL_CONNECTED) {
				Serial.println("Unable to connect WiFi");
				return;
			}
		}

		float voltageA = getRTU(mainMeterNode, 0x0000);
		float voltageB = getRTU(mainMeterNode, 0x0002);
		float voltageC = getRTU(mainMeterNode, 0x0004);

		float currentA = getRTU(mainMeterNode, 0x0008);
		float currentB = getRTU(mainMeterNode, 0x000A);
		float currentC = getRTU(mainMeterNode, 0x000C);

		float fullPower = getRTU(mainMeterNode, 0x0010);
		float powerA = getRTU(mainMeterNode, 0x0012);
		float powerB = getRTU(mainMeterNode, 0x0014);
		float powerC = getRTU(mainMeterNode, 0x0016);

		float freq = getRTU(mainMeterNode, 0x0036);
		float totalEnergy = getRTU(mainMeterNode, 0x0100);

		// ModbusMaster can't read the second meter without some delay.
		delay(100);

		float totalSolarEnergy = getRTU(solarMeterNode, 0x0100);
		float powerS = getRTU(solarMeterNode, 0x0012);
		float voltageS = getRTU(solarMeterNode, 0x0000);
		float currentS = getRTU(solarMeterNode, 0x0008);

		int8_t rssiValue = WiFi.RSSI();
		float calculatedFullPower = fullPower-powerS;
		float calculatedExportPower = 0;

		if (calculatedFullPower < 0) {
			calculatedExportPower = -calculatedFullPower;
			calculatedFullPower = 0;
		}

		Serial.print("Full power ");
		Serial.println(calculatedFullPower);

		Serial.print("Total Energy ");
		Serial.println(totalEnergy-totalSolarEnergy);

		Serial.print("Solar power ");
		Serial.println(powerS);

		Serial.print("Solar Energy ");
		Serial.println(totalSolarEnergy);

		voltageASensor.setValue(voltageA);
		voltageBSensor.setValue(voltageB);
		voltageCSensor.setValue(voltageC);
		voltageSSensor.setValue(voltageS);

		currentASensor.setValue(currentA);
		currentBSensor.setValue(currentB);
		currentCSensor.setValue(currentC - currentS);
		currentSSensor.setValue(currentS);

		fullPowerSensor.setValue(calculatedFullPower);
		exportPowerSensor.setValue(calculatedExportPower);

		powerASensor.setValue(powerA);
		powerBSensor.setValue(powerB);
		powerCSensor.setValue(powerC - powerS);
		powerSSensor.setValue(powerS);

		freqSensor.setValue(freq);
		totalEnergySensor.setValue(totalEnergy - totalSolarEnergy);
		totalSolarEnergySensor.setValue(totalSolarEnergy);

		rssi.setValue(rssiValue);

		distance dist = getDistance();

		distanceGreenSensor.setValue(dist.green);
		distanceYellowSensor.setValue(dist.yellow);
		distanceRedSensor.setValue(dist.red);
	}
	mqtt.loop();
}
