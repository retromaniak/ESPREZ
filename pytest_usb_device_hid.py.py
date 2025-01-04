import usb.core
import usb.util
import time

# VID i PID twojego urządzenia
VID = 0x0B49  # Twój VID
PID = 0x064F  # Twój PID

# Znalezienie urządzenia
dev = usb.core.find(idVendor=VID, idProduct=PID)

if dev is None:
    raise ValueError("Nie znaleziono urządzenia USB!")

# Ustawienie konfiguracji urządzenia
dev.set_configuration()

# Konfiguracja danych USB
bmRequestType = 0x41  # Host-to-device, vendor request, interface recipient
bRequest = 0x00       # Request 0x00 (Vendor-specific)
wIndex = 0x030E       # Wartość wIndex (z tej wiadomości)
wLength = 0x00        # Brak danych do wysłania (wLength = 0)
data_to_send = []     # Brak dodatkowych danych

# Funkcja do wysyłania danych USB
def send_vibration_level(level):
    wValue = level  # Wartość wValue (poziom wibracji)
    try:
        dev.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, data_to_send)
        print(f"Wysłano dane: wValue={wValue} (poziom wibracji={level})")
    except Exception as e:
        print(f"Błąd podczas wysyłania danych: {e}")

# Parametry testu
delay_between_tests = 0.05  # Opóźnienie między kolejnymi zmianami poziomów wibracji (50 ms)

# Przetwarzanie poziomów wibracji od 0 do 255 i z powrotem
try:
    while True:
        # Od 0 do 255
        for level in range(256):
            send_vibration_level(level)
            time.sleep(delay_between_tests)  # Opóźnienie między wysyłaniem poziomów

        # Od 255 do 0
        for level in range(255, -1, -1):
            send_vibration_level(level)
            time.sleep(delay_between_tests)  # Opóźnienie między wysyłaniem poziomów

except KeyboardInterrupt:
    print("Przerwano przez użytkownika.")
