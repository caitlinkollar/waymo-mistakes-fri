#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import sounddevice as sd
import numpy as np
import threading

class MicAudioPublisher(Node):
    def __init__(self):
        super().__init__('mic_audio_publisher')
        self.publisher_ = self.create_publisher(UInt8MultiArray, '/mic_audio', 10)

        self.rate = 16000  
        self.channels = 1
        self.chunk = 1024  

        threading.Thread(target=self.stream_audio, daemon=True).start()
        self.get_logger().info("Microphone publisher node started")

    def callback(self, indata, frames, time, status):

        if status:
            self.get_logger().warn(str(status))

        audio_bytes = indata.tobytes()
        msg = UInt8MultiArray()
        msg.data = list(audio_bytes)
        self.publisher_.publish(msg)

    def stream_audio(self):
        with sd.InputStream(samplerate=self.rate, channels=self.channels, dtype='int16',
                            blocksize=self.chunk, callback=self.callback):
            self.get_logger().info("Streaming microphone audio...")
            while rclpy.ok():
                sd.sleep(1000)

def main(args=None):
    rclpy.init(args=args)
    node = MicAudioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
