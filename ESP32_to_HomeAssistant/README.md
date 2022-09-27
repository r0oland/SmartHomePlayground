## Other Options to check out

## Using ESP Home 

### Installing ESPHome Manually

- `conda create -n esp32`
- `conda activate esp32`
- `mamba upgrade --all`
- `mamba install wheel`
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