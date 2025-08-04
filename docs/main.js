// Ověření načtení esptool-js
if (!window.ESPTool) {
    console.error('ESPTool is not loaded. Please ensure the esptool-js library is included.');
    document.getElementById('status').textContent = 'Error: esptool-js library failed to load';
}

// Odkaz na kombinovaný binární soubor pro TTGO LoRa V1
const firmwareUrl = 'https://github.com/ok5tvr/LoRa_RX_Igate/raw/main/binaries/firmware.bin';

const connectButton = document.getElementById('connectButton');
const flashButton = document.getElementById('flashButton');
const flashWithConfigButton = document.getElementById('flashWithConfig');
const status = document.getElementById('status');
const saveConfigButton = document.getElementById('saveConfig');
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
        flashWithConfigButton.disabled = false;
    } catch (error) {
        status.textContent = `Error: ${error.message}`;
        console.error(error);
    }
});

// Flashování firmwaru (výchozí config.txt)
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

// Flashování firmwaru s uživatelským config.txt
flashWithConfigButton.addEventListener('click', async () => {
    if (!window.ESPTool) {
        status.textContent = 'Error: esptool-js library is not available';
        return;
    }
    if (!port) {
        status.textContent = 'Error: Not connected to TTGO LoRa V1';
        return;
    }

    status.textContent = 'Status: Flashing firmware with custom config.txt...';
    try {
        // Nejprve nahrajeme kombinovaný firmware
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

        // Poté nahrajeme uživatelský config.txt přes sériový port
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

        const encoder = new TextEncoder();
        const data = encoder.encode(`CONFIG_START${configContent}CONFIG_END`);
        const writer = port.writable.getWriter();
        await writer.write(data);
        writer.releaseLock();

        // Čtení odpovědi od TTGO LoRa V1
        const reader = port.readable.getReader();
        let response = '';
        const timeout = setTimeout(() => {
            reader.cancel();
            status.textContent = 'Error: No response from TTGO LoRa V1';
        }, 5000);

        while (true) {
            const { value, done } = await reader.read();
            if (done) break;
            response += new TextDecoder().decode(value);
            if (response.includes('CONFIG_SAVED')) {
                status.textContent = 'Status: Firmware and custom config.txt successfully flashed!';
                clearTimeout(timeout);
                break;
            } else if (response.includes('CONFIG_ERROR')) {
                status.textContent = 'Error: Failed to upload custom config.txt';
                clearTimeout(timeout);
                break;
            }
        }
        reader.releaseLock();
    } catch (error) {
        status.textContent = `Error: ${error.message}`;
        console.error(error);
    }
});