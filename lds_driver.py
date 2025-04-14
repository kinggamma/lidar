
import asyncio
import serial_asyncio

class LDSDriver:
    def __init__(self, port, baudrate=230400):
       
        self.port = port
        self.baudrate = baudrate
        self.reader = None
        self.writer = None

    async def connect(self):
       
        try:
            self.reader, self.writer = await serial_asyncio.open_serial_connection(
                url=self.port, baudrate=self.baudrate
            )
            # Send start command to begin motor rotation
            self.writer.write(b"b")
            await self.writer.drain()
            print("Connected to LDS on port", self.port)
        except Exception as e:
            print("Error connecting to serial port:", e)
            raise

    async def disconnect(self):
     
        if self.writer:
            self.writer.write(b"e")
            await self.writer.drain()
            self.writer.close()
            await self.writer.wait_closed()
            print("Disconnected from LDS")

    async def read_frame(self):
        """
        Waits for and reads one complete frame.
        The protocol is assumed to be:
         - A frame starts with two sync bytes: 0xFA then 0xA0
         - The complete frame length is 2520 bytes (i.e. 60 packets of 42 bytes each).
        """
        # Sync: look for 0xFA, 0xA0
        while True:
            first_byte = await self.reader.readexactly(1)
            if first_byte[0] == 0xFA:
                second_byte = await self.reader.readexactly(1)
                if second_byte[0] == 0xA0:
                    # Found the start sequence; now read the remaining 2518 bytes
                    rest = await self.reader.readexactly(2518)
                    frame = b'\xFA' + second_byte + rest
                    return frame
                else:
                    # Continue scanning if the sequence doesn't match
                    continue

    def parse_frame(self, frame):
        """
        Parses a full 2520-byte frame into an array of 360 distance values (in meters).
        The frame is assumed to contain 60 packets of 42 bytes each.
        Each packet header should be: 0xFA, (0xA0 + packet_index)
        Each packet contains 6 measurements. In each measurement group:
          - Bytes [0] and [1] (of the 4-byte block) are the intensity (not used here)
          - Bytes [2] and [3] form the distance (in mm), where:
                distance = (byte3 << 8) + byte2
        The measurements are stored in reverse order (i.e. index = 359 - (packet_idx * 6 + measure_idx)).
        """
        if len(frame) != 2520:
            raise ValueError("Invalid frame length: expected 2520 bytes, got %d" % len(frame))

        distances = [0] * 360
        # There are 60 packets of 42 bytes each
        for packet_idx in range(60):
            offset = packet_idx * 42
            packet = frame[offset : offset + 42]
            expected_second_byte = 0xA0 + packet_idx
            if packet[0] != 0xFA or packet[1] != expected_second_byte:
                # Skip this packet if the header does not match
                continue
            # Each packet contains 6 measurement groups
            for measure_idx in range(6):
                base = 4 + measure_idx * 6  # measurements start at byte 4
                if base + 3 < len(packet):
                    # Extract four bytes: intensity (bytes 0 & 1) and range (bytes 2 & 3)
                    # Here we use only the range.
                    byte2 = packet[base + 2]
                    byte3 = packet[base + 3]
                    range_mm = (byte3 << 8) + byte2  # range in millimeters
                    range_m = range_mm / 1000.0       # convert to meters
                    # Calculate measurement index (reverse order as in official code)
                    index = packet_idx * 6 + measure_idx
                    distances[359 - index] = range_m
        return distances

    async def poll(self):
        """
        Polls one full frame from the sensor, parses it, and returns the list of distances.
        """
        frame = await self.read_frame()
        distances = self.parse_frame(frame)
        return distances

# Example usage:
if __name__ == '__main__':
    async def main():
        # Adjust the port for macOS; it might be like '/dev/tty.usbserial-XXXX'
        driver = LDSDriver(port='/dev/ttyUSB0')
        await driver.connect()
        try:
            while True:
                distances = await driver.poll()
                # For testing, just print the first 10 distances:
                print("Distances (first 10):", distances[:10])
                await asyncio.sleep(0.1)  # small delay to avoid spamming
        except KeyboardInterrupt:
            print("Stopping polling...")
        finally:
            await driver.disconnect()

    asyncio.run(main())
