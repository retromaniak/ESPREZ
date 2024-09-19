import usb.core
import usb.util

# VID i PID twojego urządzenia
VID = 0x0B49  # Twój VID
PID = 0x064F  # Twój PID

# Znalezienie urządzenia
dev = usb.core.find(idVendor=VID, idProduct=PID)

if dev is None:
    raise ValueError("Nie znaleziono urządzenia USB!")

# Ustawienie konfiguracji urządzenia
dev.set_configuration()

# Przygotowanie danych do wysłania (zgodnie z żądaniem 41 00 8E 8E 0E 03 00 00)
bmRequestType = 0x41  # Host-to-device, vendor request, interface recipient
bRequest = 0x00       # Request 0x00 (Vendor-specific)
wValue = 0x8E8E       # Wartość wValue
wIndex = 0x030E       # Wartość wIndex (z tej wiadomości)
wLength = 0x00        # Brak danych do wysłania (wLength = 0)

# Brak danych do wysyłania, ponieważ wLength = 0
data_to_send = []

# Wysyłanie danych za pomocą kontrolnego transferu USB (setup transfer)
try:
    dev.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, data_to_send)
    print(f"Wysłano dane z setup: bmRequestType={bmRequestType}, bRequest={bRequest}, wValue={hex(wValue)}, wIndex={hex(wIndex)}")
except Exception as e:
    print(f"Błąd podczas wysyłania danych: {e}")
