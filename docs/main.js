// Ověření načtení esptool-js
if (!window.ESPTool) {
    console.error('ESPTool is not loaded. Please ensure the esptool-js library is included.');
    document.getElementById('status').textContent = 'Error: esptool-js library failed to load';
}

// Odkaz na kombinovaný binární soubor pro TTGO LoRa V1
const firmwareUrl = 'https://github.com/ok5tvr/LoRa_RX_Igate/raw/main/binaries/firmware.bin';

const connectButton = document.getElementById('connectButton');
const flashButton = document.getElementById('flashButton');
const status = document.getElementById('status');
let port;

// Připojení k TTGO LoRa V1
connectButton.addEventListener('click', async () => {
    if (!window.ESPTool) {
        status.textContent = 'Error: esptool-js library is not available';
        return;
    }
    try {
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });
        status.textContent = 'Status: Connected to TTGO LoRa V1';
        flashButton.disabled = false;
    } catch (error) {
        status.textContent = `Error: ${error.message}`;
        console.error(error);
    }
});

// Flashování firmwaru
flashButton.addEventListener('click', async () => {
    if (!window.ESPTool) {
        status.textContent = 'Error: esptool-js library is not available';
        return;
    }
    status.textContent = 'Status: Flashing firmware...';
    try {
        const response = await fetch(firmwareUrl);
        if (!response.ok) {
            throw new Error(`Failed to fetch firmware: ${response.statusText}`);
        }
        const firmware = await response.arrayBuffer();
        const esptool = new window.ESPTool.ESPLoader(port);
        await esptool.connect();
        await esptool.flash(firmware, {
            chip: 'ESP32',
            flashSize: '4MB',
            progress: (percent) => {
                status.textContent = `Status: Flashing firmware (${percent}%)...`;
            }
        });
        await esptool.reset();
        status.textContent = 'Status: Firmware successfully flashed!';
    } catch (error) {
        status.textContent = `Error: ${error.message}`;
        console.error(error);
    }
});