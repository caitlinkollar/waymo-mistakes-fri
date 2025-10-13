import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from llama_cpp import Llama


class LLM_Brain(Node):

    def __init__(self):
        super().__init__('llm_brain')
        self.subscription = self.create_subscription(
            String,
            'transcription',
            self.listener_callback,
            10)
        self.subscription 
        self.llm = Llama.from_pretrained(
            # Change the model later. This model isn't really great.
            repo_id="google/gemma-7b-it",
            filename="gemma-7b-it.gguf",
            n_gpu_layers=-1,
            verbose=False
        )

    def listener_callback(self, msg):
        output = self.llm(
            msg.data,
            max_tokens= 30, # This defines the max amount of tokens.
        )
        print(output["choices"][0]["text"] )


def main(args=None):
    rclpy.init(args=args)
    brain = LLM_Brain()
    rclpy.spin(brain)
    brain.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()