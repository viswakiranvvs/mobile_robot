#!/home/robot2/miniconda3/envs/env_isaacsim/bin/python
import re
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
from transformers import Qwen3VLForConditionalGeneration, Qwen3VLProcessor, BitsAndBytesConfig
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
        model_id = "Qwen/Qwen3-VL-8B-Instruct"

        bnb_config = BitsAndBytesConfig(
            load_in_4bit=True,
            # bnb_4bit_use_double_quant=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.bfloat16
        )


        base_model = Qwen3VLForConditionalGeneration.from_pretrained(
            model_id,
            device_map="auto",
            torch_dtype=torch.float16,
            quantization_config=bnb_config,
            local_files_only=True
        )

        self.model = PeftModel.from_pretrained(
            base_model,
            "viswakiranvvs/Drone-qwen3-8b-Grip-Camera",
            # local_files_only=True
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
            10
        )

        self.GripCamera = self.create_subscription(
            Image,
            '/grip_camera_rgb',
            self.GripCamera_callback,
            10
        )

        # Initialize the current target position
        self.current_pose = PoseStamped()
        self.current_pose.pose.position = Point(x=0.0, y=0.0, z=1.0) # Start at 2m altitude
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
        self.system_message="""You are a vision-language control model for an autonomous drone equipped with a gripper.
        Your task is to analyze the image from the drone’s camera and decide the next low-level action required to pick up the target object.
        when near to object attempt a gripper close action.
        Drone is facing forward. +x is right, +y is forward, and +z is up. -x is left, -y is back, and -z is down.
        You must output ONLY a valid JSON object describing the action.

        Allowed action types:
        - drone movement
        - gripper control

        Output JSON schema:
        {
        "type": "drone" | "gripper",
        "activity": "move" | "open" | "close",
        "displacement": {
            "x": float,
            "y": float,
            "z": float,
            "yaw": float
        }
        }

        Rules:
        - Output JSON only. No explanation.
        - If alignment with the object is needed, output a drone move action.
        - If the gripper is positioned correctly around the object, output a gripper close action.
        - Keep movements small and precise.
        - Use zeros for displacement when controlling the gripper.
        - Ensure the JSON is valid.
        """


        self.query = "Predict the next control action for the drone to pick the object. If object is close enough, pick it"
    
    def goal_subscriber_callback(self, msg):
        self.current_pose = msg
    
    def rightGripCamera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.currentImage['rightGripCamera'] = cv_image

    def leftGripCamera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.currentImage['leftGripCamera'] = cv_image

    def extract_json(self,text):
        text = re.sub(r"```json", "", text)
        text = re.sub(r"```", "", text)
        return text.strip()

    def GripCamera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.currentImage['GripCamera'] = cv_image
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # Convert to PIL
        pil_image = PILImage.fromarray(rgb_image)
        # if 'rightGripCamera' not in self.currentImage:
        #     self.get_logger().warning("Right grip camera image not available yet. Skipping this frame.")
        #     return
        # right_rgb_image = cv2.cvtColor(self.currentImage['rightGripCamera'], cv2.COLOR_BGR2RGB)
        # right_pil_image = PILImage.fromarray(right_rgb_image)
        sample = self.create_sample(pil_image)
        output = self.generate_text_from_sample(sample)
        
        print(output)
        pil_image.save("debug_image.jpg")
        output_json = json.loads(self.extract_json(output))
        self.write_to_pose(output_json)
    
    def create_sample(self, image, rightImage=None):
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
                    # {
                    #     "type": "image",
                    #     "image": rightImage,
                    # },
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
        
    

    def write_to_pose(self, msg):
        # Get the current time
        now = self.get_clock().now()
        self.current_pose.header.stamp = now.to_msg()
        self.current_pose.pose.position.x += msg['displacement']['x']
        self.current_pose.pose.position.y += msg['displacement']['y']
        self.current_pose.pose.position.z -= abs(msg['displacement']['z'])
        # self.current_pose.pose.orientation.yaw += msg['displacement']['yaw']

        current_yaw = 2.0 * math.atan2(self.current_pose.pose.orientation.z, 
                                     self.current_pose.pose.orientation.w)
        self.current_pose.pose.orientation.z = math.sin(current_yaw / 2.0)
        self.current_pose.pose.orientation.w = math.cos(current_yaw / 2.0)
        self.goal_publisher.publish(self.current_pose)
        self.get_logger().info(f"Published new goal pose: {self.current_pose.pose.position}\n")
        delay = 2.0 
        # self.get_logger().info(f"Waiting for {delay:.2f} seconds before next update...")
        import time
        time.sleep(delay)

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
