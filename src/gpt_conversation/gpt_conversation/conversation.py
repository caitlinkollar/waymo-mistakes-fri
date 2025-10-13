import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os

class OpenAISubscriber(Node):
    def __init__(self):
        super().__init__('openai_subscriber')
        openai.api_key = os.getenv("OPENAI_API_KEY")
        if openai.api_key is None:
            self.get_logger().error("OPENAI_API_KEY environment variable not set.")
            raise RuntimeError("Missing OpenAI API key")
        
        self.subscription = self.create_subscription(
            String,
            '/transcription',
            self.transcription_callback,
            10
        )
        
        # Create publisher for OpenAI responses
        self.response_pub = self.create_publisher(String, '/openai_response', 10)
        
        self.get_logger().info("OpenAI subscriber node started and listening on /transcription")

    def transcription_callback(self, msg: String):
        text = msg.data
        self.get_logger().info(f"Received transcription: {text}")

        try:
            response = openai.ChatCompletion.create(
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": "You are a helpful assistant."},
                    {"role": "user", "content": text}
                ]
            )
            answer = response.choices[0].message.content.strip()
            self.get_logger().info(f"OpenAI response: {answer}")

            # Publish response
            out_msg = String()
            out_msg.data = answer
            self.response_pub.publish(out_msg)
            self.get_logger().info("Published OpenAI response on /openai_response")

        except Exception as e:
            self.get_logger().error(f"OpenAI API call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = OpenAISubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
