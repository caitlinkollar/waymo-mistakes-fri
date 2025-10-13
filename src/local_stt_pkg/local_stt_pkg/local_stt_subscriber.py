#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import speech_recognition as sr
import io
import wave
import numpy as np

class LocalSTTSubscriber(Node):
    def __init__(self):
        super().__init__('local_stt_subscriber')

        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/mic_audio',
            self.audio_callback,
            10)

        self.transcription_pub = self.create_publisher(String, '/transcription', 10)

        self.get_logger().info("Local STT subscriber node started")

        self.recognizer = sr.Recognizer()

        # Audio config (must match your microphone publisher)
        self.rate = 16000
        self.channels = 1
        self.sample_width = 2  # bytes (16 bits)

        # Buffer to accumulate audio before processing
        self.audio_buffer = bytearray()

        # Process after this many bytes (e.g., ~1 second of audio)
        self.buffer_threshold = self.rate * self.sample_width  # 1 second

    def audio_callback(self, msg: UInt8MultiArray):
        # Append incoming audio data
        self.audio_buffer.extend(msg.data)

        # If we've accumulated enough audio, process it
        if len(self.audio_buffer) >= self.buffer_threshold:
            self.process_audio(bytes(self.audio_buffer))
            self.audio_buffer.clear()

    def process_audio(self, audio_bytes):
        try:
            # Convert raw PCM data to WAV format in memory
            wav_io = io.BytesIO()
            with wave.open(wav_io, 'wb') as wf:
                wf.setnchannels(self.channels)
                wf.setsampwidth(self.sample_width)
                wf.setframerate(self.rate)
                wf.writeframes(audio_bytes)
            wav_io.seek(0)

            # Use speech_recognition on the in-memory WAV file
            with sr.AudioFile(wav_io) as source:
                audio_data = self.recognizer.record(source)
                text = self.recognizer.recognize_sphinx(audio_data)
                self.get_logger().info(f"Recognized: {text}")

                # Publish the transcription
                msg = String()
                msg.data = text
                self.transcription_pub.publish(msg)

        except sr.UnknownValueError:
            self.get_logger().info("Could not understand audio")
        except sr.RequestError as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LocalSTTSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
