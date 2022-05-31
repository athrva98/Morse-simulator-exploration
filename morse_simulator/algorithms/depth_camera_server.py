import socket
import json
import threading
import base64
from morse_simulator.algorithms.config import config




class DepthCameraServer(threading.Thread):
    """
    Class which receives data from DepthCamera by built-in 'socket' interface, decodes it from json, utf-8 and base64
    and makes it available for user
    """

    def __init__(self, host, port):
        """
        Initializes socket and opens data stream.
        :param host: address of a streaming socket
        :param port: port on which is the stream
        """
        self.endline = '\n'
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.sock.connect((host, port))

        self.current_data = 0
        self.current_3D_points = 0
        self.is_new_data = False

    def recv_end(self):
        """
        This method is used to obtain only one object from socket.
        Sometimes it still can return not valid format if it starts receiving in the middle of transmission.
        :return: string with json formatting
        """
        total_data = []

        while True:
            data = self.sock.recv(65536 * (config.num_robots_per_senv+2))
            data = data.decode('utf-8')

            # If there is endline in message, it means this is the end of this message
            if self.endline in data:
                single_data = data[:data.find(self.endline)]
                total_data.append(single_data)
                break

            total_data.append(data)

            if len(total_data) > 1:
                # check if end_of_data was split
                last_pair = total_data[-2]+total_data[-1]
                if self.endline in last_pair:
                    total_data[-2] = last_pair[:last_pair.find(self.endline)]
                    total_data.pop()
                    break

        return ''.join(total_data)

    def receive_data(self):
        """
        Keeps the actual sensor data up-to-date. Should be run as thread.
        It also prints info to terminal if received data is not valid.
        When full data sample is received and put together, flag is_new_data is set to True
        """
        while True:

            data = self.recv_end()

            if len(data) > 0:

                try:
                    self.current_data = json.loads(data)
                    self.current_3D_points = base64.b64decode(self.current_data['points'])
                    self.is_new_data = True

                except:
                    # print('cannot load DepthCamera data')
                    pass

    def get_points(self):
        """
        Returns the cloud of 3D points as a bytes object, decoding is needed
        :return: prints info to terminal if wanted data is available
        """
        try:
            return self.current_3D_points
        except:
            print('no such current_image')

    def get_all(self):
        """
        Return the whole sample of data from stream (data is in form of python dict)
        :return: python dict with data from sensor
        """
        try:
            return self.current_data
        except:
            print('No data received from sensor')

    def run(self):
        """
        Runs the receiving method as thread.
        :return:
        """
        receiver = threading.Thread(target=self.receive_data)
        # Setting daemon to True means that this Thread will be terminated when the main program ends.
        receiver.daemon = True
        receiver.start()


# function rotating view from DepthCamera
def rotation(x, y, z, yaw, pitch, roll):
    import math
    import scipy
    import numpy as np
    """
    Function that rotates about three orthogonal axes. These rotations will be referred to as yaw, pitch, and roll.
    :param
        x(float): x coordinate of the sensor, in sensor coordinate, in meter
        y(float): y coordinate of the sensor, in sensor coordinate, in meter
        z(float): z coordinate of the sensor, in sensor coordinate, in meter
        yaw(float): rotation around the Z axis of the sensor, in radian
        pitch(float): rotation around the Y axis of the sensor, in radian
        roll(float): rotation around the X axis of the sensor, in radian
    :return:
        xyz[0](float): x coordinate of the sensor, in world coordinate, in meter
        xyz[1](float): y coordinate of the sensor, in world coordinate, in meter
        xyz[2](float): z coordinate of the sensor, in world coordinate, in meter
    """
    camera_translate = - math.pi / 8
    pitch += camera_translate
    xyz = scipy.array([x, y, z])
    yaw_cos = math.cos(yaw)
    yaw_sin = math.sin(yaw)
    pitch_cos = math.cos(pitch)
    pitch_sin = math.sin(pitch)
    roll_cos = math.cos(roll)
    roll_sin = math.sin(roll)
    r1 = [yaw_cos * pitch_cos, yaw_cos * pitch_sin * roll_sin - yaw_sin * roll_cos,
          yaw_cos * pitch_sin * roll_cos + yaw_sin * roll_sin]
    r2 = [yaw_sin * pitch_cos, yaw_sin * pitch_sin * roll_sin + yaw_cos * roll_cos,
          yaw_sin * pitch_sin * roll_cos - yaw_cos * roll_sin]
    r3 = [-pitch_sin, pitch_cos * roll_sin, pitch_cos * roll_cos]
    r = scipy.array([r1, r2, r3])
    xyz = np.dot(r, xyz)

    return xyz[0], xyz[1], xyz[2]


#if __name__ == '__main__':
#    main()
