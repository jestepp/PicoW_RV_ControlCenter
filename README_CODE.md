This ZIP contains the PlatformIO project code only.

Set environment variables in your terminal before building:
WIFI_SSID, WIFI_PASS, MQTT_BROKER, MQTT_DISCOVERY (0/1), optional MQTT_USER/MQTT_PASS, MQTT_BASE_TOPIC.
Then:
  pio run -t upload
  pio device monitor
