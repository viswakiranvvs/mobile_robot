#!/home/robot2/miniconda3/envs/env_isaacsim/bin/python
import sys

print("Python used:", sys.executable)

from platform import processor
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import math
from sensor_msgs.msg import Image
from PIL import Image as PILImage
from cv_bridge import CvBridge
import json
# from huggingface_hub import login
# from datasets import load_dataset
# python -c "import sys; print(sys.executable)"

import torch
from transformers import Qwen3VLForConditionalGeneration, Qwen3VLProcessor
from qwen_vl_utils import process_vision_info
from peft import PeftModel

class QwenController(Node):

    def __init__(self):
        super().__init__('qwen_controller')
        self.bridge = CvBridge()
        # Subscribe to the joystick input
        # self.subscription = self.create_subscription(
        #     Joy,
        #     'joy', # Standard topic for joystick data
        #     self.joy_callback,
        #     10
        # )
        self.device="cuda"
        model_id = "Qwen/Qwen3-VL-2B-Instruct"


        base_model = Qwen3VLForConditionalGeneration.from_pretrained(
            model_id,
            device_map="auto",
            torch_dtype=torch.float16,
            local_files_only=True
        )

        self.model = PeftModel.from_pretrained(
            base_model,
            "viswakiranvvs/Drone-qwen3-2b-instruct-trl-sft-1",
            local_files_only=True
        )

        lora_layers = [n for n, _ in self.model.named_modules() if "lora" in n.lower()]
        self.get_logger().info("LoRA layers: " + str(len(lora_layers)))
        # .to(device)

        self.processor = Qwen3VLProcessor.from_pretrained(model_id)

        # viswakiranvvs/Drone-qwen3-8b-instruct-trl-sft

        # adapter_path = "viswakiranvvs/Drone-qwen3-2b-instruct-trl-sft-1"
        # # # adapter_path = "sergiopaniego/qwen2-7b-instruct-trl-sft-ChartQA"
        # self.model.load_adapter(adapter_path)        


        self.goal_publisher = self.create_subscription(
            PoseStamped,
            '/goal_pose', # The topic your NonlinearController listens to
            self.goal_subscriber_callback,
            10
        )
                
        # Publisher for the goal pose
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose', # The topic your NonlinearController listens to
            10
        )

        self.grip_publisher = self.create_publisher(
            PoseStamped,
            '/grip_pose', # The topic your NonlinearController listens to
            10
        )

        self.rightGripCamera = self.create_subscription(
            Image,
            '/right_grip_camera_rgb',
            self.rightGripCamera_callback,
            10
        )

        self.leftGripCamera = self.create_subscription(
            Image,
            '/left_grip_camera_rgb',
            self.leftGripCamera_callback,
            10
        )

        self.droneArmCamera = self.create_subscription(
            Image,
            '/drone_arm_camera_rgb',
            self.droneArmCamera_callback,
            1
        )

        # Initialize the current target position
        self.current_pose = PoseStamped()
        self.current_pose.pose.position = Point(x=0.0, y=0.0, z=2.0) # Start at 2m altitude
        self.current_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.5, w=0.5) # Default orientation
        self.current_pose.header.frame_id = 'world'

        self.grip_pose = PoseStamped()
        self.grip_pose.pose.position = Point(x=0.0, y=0.0, z=60.0) # Start at 2m altitude
        self.grip_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.5, w=0.5) # Default orientation
        self.grip_pose.header.frame_id = 'world'
        
        # Control parameters (adjust these for sensitivity)
        self.linear_speed = 0.5  # meters per second per joystick push
        self.angular_speed = 0.5 # radians per second per joystick push
        self.update_rate = 10  # Hz
        self.last_update = self.get_clock().now()
        self.grip_closed = False
        self.logging = False
        self.currentImage = {}
        self.event_log = []
        self.get_logger().info('Qwen Controller Node Started. Waiting for /droneCamera messages...')
        self.prev_buttons = None
        self.directory_name = ''
        self.images_dir = ''
        self.system_message = """
        You are a vision-language control model for an autonomous drone equipped with a gripper.

        Your task is to analyze the image and predict the next control action required to pick the object.

        You must output ONLY valid JSON.

        Output JSON schema:
        {
        "type": "drone" | "gripper",
        "activity": "move" | "open" | "close",
        "direction": "up" | "down" | "left" | "right" | "forward" | "backward" | "yaw_cw" | "yaw_ccw" | "none"
        }

        Rules:
        - Output JSON only.
        - Use "direction" only for drone movement.
        - Use "none" for gripper actions.
        - Movements represent small corrective steps.
        - Ensure the JSON is valid.
        """


        self.query = "Predict the next control action for the drone to pick the object."
    
    def goal_subscriber_callback(self, msg):
        self.current_pose = msg
    
    def rightGripCamera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.currentImage['rightGripCamera'] = cv_image

    def leftGripCamera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.currentImage['leftGripCamera'] = cv_image
    
    def create_sample(self, image):
        return {
        "images": [image],
        "messages": [

            {
                "role": "system",
                "content": [
                    {
                        "type": "text",
                        "text": self.system_message
                    }
                ],
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "image": image,
                    },
                    {
                        "type": "text",
                        "text": self.query,
                    }
                ],
            },
        ]
        }

    def droneArmCamera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.currentImage['droneArmCamera'] = cv_image
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # Convert to PIL
        pil_image = PILImage.fromarray(rgb_image)
        sample = self.create_sample(pil_image)
        output = self.generate_text_from_sample(sample)
        print(output)
        pil_image.save("debug_image.jpg")
        self.write_to_pose(output)
    

    def write_to_pose(self, msg):
        # Get the current time
        pass
        # now = self.get_clock().now()
        # if not self.prev_buttons:
        #     self.prev_buttons = list(msg.buttons)
        #     return
        # # Calculate time since last update to make movement frame-rate independent
        # dt = (now - self.last_update).nanoseconds / 1e9
        # if dt < (1.0 / self.update_rate):
        #     return # Skip if it's too soon to update
            
    def generate_text_from_sample(self,sample, max_new_tokens=1024, device="cuda"):
        self.model.to(device)
        # Prepare the text input by applying the chat template
        text_input = self.processor.apply_chat_template(
            sample['messages'][0:2],  # Use the sample without the system message
            tokenize=False,
            add_generation_prompt=True
        )

        # Process the visual input from the sample
        image_inputs, _ = process_vision_info(sample['messages'])

        # Prepare the inputs for the model
        model_inputs = self.processor(
            text=[text_input],
            images=image_inputs,
            return_tensors="pt",
        ).to(device)
        # .to(device)
        # .to(device)  # Move inputs to the specified device

        # Generate text with the model
        generated_ids = self.model.generate(**model_inputs, max_new_tokens=max_new_tokens)

        # Trim the generated ids to remove the input ids
        trimmed_generated_ids = [
            out_ids[len(in_ids):] for in_ids, out_ids in zip(model_inputs.input_ids, generated_ids)
        ]

        # Decode the output text
        output_text = self.processor.batch_decode(
            trimmed_generated_ids,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False
        )

        return output_text[0]  # Return the first decoded output text



def main(args=None):
    rclpy.init(args=args)
    joystick_controller = QwenController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
