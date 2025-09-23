#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import azure.cognitiveservices.speech as speechsdk

class AzureSTTSubscriber(Node):
    def __init__(self):
        super().__init__('azure_stt_subscriber')

    
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/mic_audio',
            self.audio_callback,
            10)

        
        self.transcription_pub = self.create_publisher(String, '/transcription', 10)

        
        self.rate = 16000
        self.channels = 1

        
        self.speech_key = "AZURE_SPEECH_KEY"
        self.service_region = "centralus"

        self.speech_config = speechsdk.SpeechConfig(subscription=self.speech_key, region=self.service_region)
        self.speech_config.speech_recognition_language = "en-US"

       
        self.push_stream = speechsdk.audio.PushAudioInputStream()
        audio_format = speechsdk.audio.AudioStreamFormat(samples_per_second=self.rate, bits_per_sample=16, channels=self.channels)
        audio_input = speechsdk.audio.AudioConfig(stream=self.push_stream)

        
        self.speech_recognizer = speechsdk.SpeechRecognizer(speech_config=self.speech_config, audio_config=audio_input)

       
        self.speech_recognizer.recognized.connect(self.recognized_callback)

       
        self.speech_recognizer.start_continuous_recognition()

        self.get_logger().info("Azure STT subscriber node started")

    def audio_callback(self, msg: UInt8MultiArray):
        
        audio_bytes = bytes(msg.data)

        
        self.push_stream.write(audio_bytes)

    def recognized_callback(self, evt):
       
        text = evt.result.text
        if text:
            self.get_logger().info(f"Recognized: {text}")

          
            msg = String()
            msg.data = text
            self.transcription_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AzureSTTSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
