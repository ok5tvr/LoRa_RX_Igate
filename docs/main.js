const boardFirmware = {
    ttgo: 'https://raw.githubusercontent.com/ok5tvr/LoRa_RX_Igate/main/binaries/firmware.bin'
};

const connectButton = document.getElementById('connectButton');
const flashButton = document.getElementById('flashButton');
const boardSelect = document.getElementById('boardSelect');
const status = document.getElementById('status');
const saveConfigButton = document.getElementById('saveConfig');
let port;

// Připojení k ESP32
connectButton.addEventListener('click', async () => {
    try {
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });
        status.textContent = 'Status: Connected to ESP32';
        flashButton.disabled = false;
    } catch (error) {
        status.textContent = `Error: ${error.message}`;
    }
});

// Flashování firmwaru
flashButton.addEventListener('click', async () => {
    status.textContent = 'Status: Flashing firmware...';
    try {
        const firmwareUrl = boardFirmware[boardSelect.value];
        const response = await fetch(firmwareUrl);
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
    }
});

// Generování config.txt
saveConfigButton.addEventListener('click', () => {
    const wifi_ssid = document.getElementById('wifi_ssid').value || '';
    const wifi_password = document.getElementById('wifi_password').value || '';
    const callsign = document.getElementById('callsign').value || '';
    const latitude = document.getElementById('latitude').value || '';
    const longitude = document.getElementById('longitude').value || '';
    const symbol = document.getElementById('symbol').value || '';
    const comment = document.getElementById('comment').value || '';
    const altitude = document.getElementById('altitude').value || '';
    const aprs_filter = document.getElementById('aprs_filter').value || '';
    const servername = document.getElementById('servername').value || '';
    const aprs_port = document.getElementById('aprs_port').value || '';
    const password_aprs = document.getElementById('password_aprs').value || '';
    const ip_manual = document.getElementById('ip_manual').checked ? 'true' : 'false';
    const local_ip = document.getElementById('local_ip').value || '';
    const gateway = document.getElementById('gateway').value || '';
    const subnet = document.getElementById('subnet').value || '';
    const primary_dns = document.getElementById('primary_dns').value || '';
    const secondary_dns = document.getElementById('secondary_dns').value || '';
    const digi = document.getElementById('digi').checked ? '1' : '0';
    const digi_ap = document.getElementById('digi_ap').checked ? '1' : '0';
    const ap_password = document.getElementById('ap_password').value || '';

    const configContent = `//----------------lora RX igate------------
/// ------------wifi-------------
wifi ssid <${wifi_ssid}>
wifi heslo <${wifi_password}>

/// ------- ID APRS -------------------------
call <${callsign}>
lon <${longitude}>
lat <${latitude}>
sym <${symbol}>
coment <${comment}>
Alt <${altitude}>
aprs_filter <${aprs_filter}>

///------------aprs setup----------------
servername <${servername}>
aprs_port <${aprs_port}>
password_aprs <${password_aprs}>

///---------------IP config---------------
IP_manual <${ip_manual}>
local_IP <${local_ip}>
gateway <${gateway}>
subnet <${subnet}>
primaryDNS <${primary_dns}>
secondaryDNS <${secondary_dns}>

///----------digi------------
digi <${digi}>
digi_AP <${digi_ap}>
ap_password <${ap_password}>

///--------beacon-------------
!`;
    const blob = new Blob([configContent], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'config.txt';
    a.click();
    URL.revokeObjectURL(url);
    status.textContent = 'Status: config.txt saved!';
});