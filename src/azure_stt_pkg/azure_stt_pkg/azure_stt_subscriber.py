#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import azure.cognitiveservices.speech as speechsdk
from dotenv import load_dotenv
import os

class AzureSTTSubscriber(Node):
    def __init__(self):
        super().__init__('azure_stt_subscriber')
        
        # Load environment variables
        load_dotenv()
        
        # Azure configuration
        speech_key = os.getenv("AZURE_SPEECH_KEY")
        service_region = os.getenv("AZURE_REGION")
        
        self.get_logger().info(f"Using Azure region: {service_region}")
        
        # Create Azure Speech Config
        try:
            self.speech_config = speechsdk.SpeechConfig(
                subscription=speech_key, 
                region=service_region
            )
            self.speech_config.speech_recognition_language = "en-US"
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Azure Speech Config: {e}")
            return
        
        # Audio stream configuration
        self.rate = 16000
        self.channels = 1
        
        # Create push stream for audio input
        audio_format = speechsdk.audio.AudioStreamFormat(
            samples_per_second=self.rate,
            bits_per_sample=16,
            channels=self.channels
        )
        self.push_stream = speechsdk.audio.PushAudioInputStream(stream_format=audio_format)
        audio_config = speechsdk.audio.AudioConfig(stream=self.push_stream)
        
        # Create speech recognizer
        self.speech_recognizer = speechsdk.SpeechRecognizer(
            speech_config=self.speech_config,
            audio_config=audio_config
        )
        
        # Connect event handlers
        self.speech_recognizer.recognizing.connect(self.recognizing_callback)
        self.speech_recognizer.recognized.connect(self.recognized_callback)
        self.speech_recognizer.canceled.connect(self.canceled_callback)
        self.speech_recognizer.session_started.connect(self.session_started_callback)
        self.speech_recognizer.session_stopped.connect(self.session_stopped_callback)
        
        # ROS2 subscriber for audio data
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/mic_audio',
            self.audio_callback,
            10
        )
        
        # ROS2 publisher for transcriptions
        self.transcription_pub = self.create_publisher(String, '/transcription', 10)
        
        # ROS2 subscriber for injected text from media_pipe
        self.media_pipe_sub = self.create_subscription(
            String,
            '/media_pipe/text_output',  # Make sure this matches actual topic name
            self.media_pipe_callback,
            10
        )

        # Start continuous recognition
        self.speech_recognizer.start_continuous_recognition()
        
        self.get_logger().info("Azure STT subscriber node started and listening...")

    def audio_callback(self, msg: UInt8MultiArray):
        """Receives audio data from ROS topic and feeds it to Azure"""
        try:
            audio_bytes = bytes(msg.data)
            self.push_stream.write(audio_bytes)
        except Exception as e:
            self.get_logger().error(f"Error in audio callback: {e}")
    
    def recognizing_callback(self, evt):
        """Called during recognition (partial results)"""
        if evt.result.text:
            self.get_logger().info(f"Recognizing: {evt.result.text}")
    
    def recognized_callback(self, evt):
        """Called when speech is fully recognized"""
        if evt.result.reason == speechsdk.ResultReason.RecognizedSpeech:
            text = evt.result.text
            self.get_logger().info(f"✓ Recognized: {text}")
            
            # Publish transcription to ROS topic
            msg = String()
            msg.data = text
            self.transcription_pub.publish(msg)
            
        elif evt.result.reason == speechsdk.ResultReason.NoMatch:
            self.get_logger().warn(f"No speech recognized: {evt.result.no_match_details}")
    
    def canceled_callback(self, evt):
        """Called if recognition is canceled"""
        self.get_logger().error(f"Recognition canceled: {evt.result.reason}")
        
        if evt.result.reason == speechsdk.ResultReason.Error:
            cancellation = speechsdk.CancellationDetails(evt.result)
            self.get_logger().error(f"Cancellation reason: {cancellation.reason}")
            self.get_logger().error(f"Error code: {cancellation.error_code}")
            self.get_logger().error(f"Error details: {cancellation.error_details}")
            
            if cancellation.error_code == speechsdk.CancellationErrorCode.ConnectionFailure:
                self.get_logger().error("Connection failed - check your internet connection")
            elif cancellation.error_code == speechsdk.CancellationErrorCode.AuthenticationFailure:
                self.get_logger().error("Authentication failed - check your Azure key and region")
    
    def session_started_callback(self, evt):
        """Called when recognition session starts"""
        self.get_logger().info("Speech recognition session started")
    
    def session_stopped_callback(self, evt):
        """Called when recognition session stops"""
        self.get_logger().info("Speech recognition session stopped")

    def media_pipe_callback(self, msg: String):
        """Receives text from media_pipe and injects it into transcription output"""
        injected_text = msg.data.strip()
        if injected_text:
            self.get_logger().info(f"Injected from media_pipe: {injected_text}")
            out_msg = String()
            out_msg.data = f"[Injected] {injected_text}"  # Optional tag
            self.transcription_pub.publish(out_msg)
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        try:
            self.speech_recognizer.stop_continuous_recognition()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AzureSTTSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()