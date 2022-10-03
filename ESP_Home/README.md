## Other Options to check out

## Using ESP Home 

### Components to try:

- [Power Supply](https://esphome.io/components/power_supply.html)

### General Things to do / keep in mind

- apply some [filtering to sensor](https://esphome.io/components/sensor/index.html#sensor-filters) data before sedning 
- fuse sensor data using [Kalman filter-based sensor fusion
](https://esphome.io/components/sensor/kalman_combinator.html#kalman-filter-based-sensor-fusion)

### Installing ESPHome Manually

- `conda create -n esp32`
- `conda activate esp32`
- `mamba upgrade --all`
- `mamba install wheel`
- `mamba install pillow` - for using fonts in display
- `pip3 install esphome`

Check for working esp home:
`esphome version`

Update to latest version
- `conda upgrade conda`
- `conda activate esp32`
- `mamba upgrade --all`
- `pip3 install -U esphome`

See [also here](https://esphome.io/guides/installing_esphome.html) for other platforms etc.

### Getting started

- run the wizard to create an example yaml file:
`esphome wizard livingroom.yaml` 

- connect the device physically and upload the code for the first time

### Direct ARDUINO / ESP to Home Assistant using MQTT

- https://github.com/dawidchyrzynski/arduino-home-assistant