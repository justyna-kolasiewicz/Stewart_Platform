import serial


class ServoController:
    def __init__(self, initial_positions, usb_port):
        self.__initial_positions = initial_positions
        self.__usb_port = usb_port
        self.__connection = serial.Serial(self.__usb_port, 9600)
        
    def __target_low(self, value):
        value = value * 4
        return int(value) & 0x7f

    def __target_high(self, value):
        value = value * 4
        return (int(value) >> 7) & 0x7f

    def set_servo_motors_angles(self, servo_motors_angles):
        query = []
        for num, [initial_position, angle] in enumerate(zip(self.__initial_positions, servo_motors_angles)):
            position = round((initial_position + 100 * angle / 9) * 4) / 4
            query.extend([132, num, self.__target_low(position), self.__target_high(position)])
        self.__connection.write(serial.to_bytes(query))

    def close_connection(self):
        self.__connection.close()
