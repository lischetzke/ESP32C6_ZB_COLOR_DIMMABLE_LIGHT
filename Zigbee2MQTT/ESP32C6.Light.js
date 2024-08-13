const {deviceEndpoints, identify, light} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['ESP32C6.Light'],
    model: 'ESP32C6.Light',
    vendor: 'Espressif',
    description: 'Modified auto gen definition. Effects and Power-on behavior do not work, check ESP32C6.Light.Manual.js or ESP32C6.Light2.js',
    extend: [deviceEndpoints({"endpoints":{"10":10,"11":11}}), identify(), light({"color":true}), light({"colorTemp":{"range":[150,500]},"color":{"modes":["xy","hs"],"enhancedHue":true}})],
    meta: {"multiEndpoint":true},
};

module.exports = definition;
