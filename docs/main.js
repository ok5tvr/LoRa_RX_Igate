const boardFirmware = {
    ttgo: 'https://github.com/ok5tvr/LoRa_RX_Igate/raw/main/binaries/firmware_ttgo.bin',
    heltec: 'https://github.com/ok5tvr/LoRa_RX_Igate/raw/main/binaries/firmware_heltec.bin',
    esp32: 'https://github.com/ok5tvr/LoRa_RX_Igate/raw/main/binaries/firmware_esp32.bin'
};

const connectButton = document.getElementById('connectButton');
const flashButton = document.getElementById('flashButton');
const boardSelect = document.getElementById('boardSelect');
const status = document.getElementById('status');
let port;

connectButton.addEventListener('click', async () => {
    try {
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });
        status.textContent = 'Stav: Připojeno k ESP32';
        flashButton.disabled = false;
    } catch (error) {
        status.textContent = `Chyba: ${error.message}`;
    }
});

flashButton.addEventListener('click', async () => {
    status.textContent = 'Stav: Nahrávání firmwaru...';
    try {
        const firmwareUrl = boardFirmware[boardSelect.value];
        const response = await fetch(firmwareUrl);
        const firmware = await response.arrayBuffer();
        const esptool = new window.ESPTool.ESPLoader(port);
        await esptool.connect();
        await esptool.flash(firmware, { chip: 'ESP32', flashSize: '4MB' });
        await esptool.reset();
        status.textContent = 'Stav: Firmware úspěšně nahrán!';
    } catch (error) {
        status.textContent = `Chyba: ${error.message}`;
    }
});