import time
import serial
import pynmea2

NUM_SATS_NEEDED = 4

class SuperRoo_GPS():

    def __init__(self, port):
        self.latitude = 0.0
        self.longitude = 0.0
        self.port = port

    def main_loop(self):
        com = None
        reader = pynmea2.NMEAStreamReader()

        while True:
            if com is None:
                try:
                    com = serial.Serial(self.port, timeout=5.0)
                except serial.SerialException:
                    print('could not connect to %s' % self.port)
                    time.sleep(5.0)
                    continue

            data = com.read(16)
            for msg in reader.next(data):
                if msg:
                    try:
                        gmsg = pynmea2.parse(str(msg))
                        if int(gmsg.num_sats) >= NUM_SATS_NEEDED:
                            # We have valid data
                            print(time.time(), "lat", gmsg.latitude, "long", gmsg.longitude)
                            self.ts = time.time()
                            self.latitude = gmsg.latitude
                            self.longitude = gmsg.longitude
                    except:
                        continue

    def poll(self):
        return self.ts, self.latitude, self.longitude

