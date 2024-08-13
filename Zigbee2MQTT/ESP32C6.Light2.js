const {deviceEndpoints, identify, light} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['ESP32C6.Light'],
    model: 'ESP32C6.Light',
    vendor: 'Espressif',
    description: 'Automatically generated definition',
    extend: [deviceEndpoints({"endpoints":{"10":10}}), identify(), light({"color":true}), light({"colorTemp":{"range":[150,500]},"color":{"modes":["xy"],"enhancedHue":true}})],
    meta: {"multiEndpoint":false},
};

module.exports = definition;
