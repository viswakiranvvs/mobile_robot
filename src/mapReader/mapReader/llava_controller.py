#!/home/robot2/miniconda3/envs/env_isaacsim/bin/python
import sys
import time
from urllib import response
import requests
from tensorboard import data
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
from transformers import LlavaForConditionalGeneration, AutoProcessor, BitsAndBytesConfig
from qwen_vl_utils import process_vision_info
from peft import PeftModel

class LlavaController(Node):

    def __init__(self):
        super().__init__('llava_controller')
        self.bridge = CvBridge()
        # Subscribe to the joystick input
        # self.subscription = self.create_subscription(
        #     Joy,
        #     'joy', # Standard topic for joystick data
        #     self.joy_callback,
        #     10
        # )
        # self.device="cuda"
        # model_id = "llava-hf/llava-1.5-7b-hf"


        # base_model = LlavaForConditionalGeneration.from_pretrained(
        #     model_id,
        #     device_map="auto",
        #     torch_dtype=torch.float16,
        #     local_files_only=True
        # )

        # self.model = PeftModel.from_pretrained(
        #     base_model,
        #     "viswakiranvvs/llava-drone-lora-float",
        #     local_files_only=True
        # )

        # self.get_logger().info("Loading model in 4 bit...")

        # bnb_config = BitsAndBytesConfig(
        #     load_in_4bit=True,
        #     bnb_4bit_compute_dtype=torch.bfloat16,
        #     bnb_4bit_use_double_quant=True,
        #     bnb_4bit_quant_type="nf4"
        # )

        # base_model = LlavaForConditionalGeneration.from_pretrained(
        #     model_id,
        #     quantization_config=bnb_config,
        #     device_map="auto",
        #     local_files_only=True
        # )

        # base_model = LlavaForConditionalGeneration.from_pretrained(
        #     model_id,
        #     quantization_config=bnb_config,
        #     device_map="auto",
        #     offload_folder="offload",
        #     offload_state_dict=True,
        #     local_files_only=True
        # )

        # self.model = PeftModel.from_pretrained(
        #     base_model,
        #     "viswakiranvvs/llava-drone-lora-float",
        #     local_files_only=True
        # )
        # self.model.eval()
        # for param in self.model.parameters():
        #     param.requires_grad = False

        # lora_layers = [n for n, _ in self.model.named_modules() if "lora" in n.lower()]
        # self.get_logger().info("LoRA layers: " + str(len(lora_layers)))
        # # .to(device)

        # self.processor = AutoProcessor.from_pretrained(model_id)

        # viswakiranvvs/Drone-qwen3-8b-instruct-trl-sft

        # adapter_path = "viswakiranvvs/Drone-qwen3-2b-instruct-trl-sft-1"
        # # # adapter_path = "sergiopaniego/qwen2-7b-instruct-trl-sft-ChartQA"
        # self.model.load_adapter(adapter_path)        


        self.goal_sub = self.create_subscription(
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

        self.GripCamera = self.create_subscription(
            Image,
            '/grip_camera_rgb',
            self.GripCamera_callback,
            10
        )

        # Initialize the current target position
        self.current_pose = PoseStamped()
        self.current_pose.pose.position = Point(x=0.39, y=0.42, z=1.48) # Start at 2m altitude
        self.current_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.5, w=0.5) # Default orientation
        self.current_pose.header.frame_id = 'world'

        self.grip_pose = PoseStamped()
        self.grip_pose.pose.position = Point(x=0.0, y=0.0, z=60.0)
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
        self.get_logger().info('Llava Controller Node Started. Waiting for /droneCamera messages...')
        self.prev_buttons = None
        self.directory_name = ''
        self.isGripClosed = False
        self.images_dir = ''
        self.filePath = 'pathTaken.json'
        self.pathTaken = []
        self.system_message="""You are a vision-language control model for a drone with a gripper.

Input:
Image 1: Downward-facing gripper camera (primary alignment view).
Image 2: View from the drone arm looking downward towards gripper

Task:
Output the single next atomic action to grasp the target object.

Coordinate system:

x right, − x left

y forward, − y back

z up, − z down
yaw: + right rotate, − left rotate

Decision policy (priority order):

If visible but not centered in gripper view → small x/y correction. Do NOT descend.
If centered but far → small downward move (−z).
If centered and very close → gripper close.

Movement rules:
Movements must be small and precise (|x|,|y|,|z| ≤ 0.05 unless clearly far).
Use yaw only if x/y cannot fix alignment.
Gripper rules:

Close only when centered and within grasp distance.

When type="gripper", all displacement values must be 0.

Allowed types:

"drone" (activity="move")

"gripper" (activity="open" or "close")

Output:
Return ONLY valid JSON. No explanation. No markdown. One action only.

Schema:

{
"type": "drone" | "gripper",
"activity": "move" | "open" | "close",
"displacement": {
"x": float,
"y": float,
"z": float,
"yaw": float
}
"reason": (The reason why that action is taken)
}

        Rules:
        - Output JSON only. No explanation.
        - If alignment with the object is needed, output a drone move action.
        - Keep movements small and precise.
        - when controlling the gripper output the gripper control action.
        - Ensure the JSON is valid.
        """

        self.query = "Predict the next control action for the drone to pick the object. If object is close enough, give the gripper close command."
    
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

    def GripCamera_callback(self, msg):
        if self.isGripClosed:
            if self.current_pose.pose.position.z < 2.0:
                command={}
                command["type"] = "drone"
                command["activity"] = "move"
                command["displacement"] = {"x": 0.0, "y": 0.0, "z": 0.2, "yaw": 0.0}
                self.write_to_pose(command)
            # else:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.currentImage['GripCamera'] = cv_image
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        if 'droneArmCamera' not in self.currentImage:
            self.get_logger().warning("droneArmCamera image not available yet. Skipping this frame.")
            return
        # Convert to PIL
        pil_image = PILImage.fromarray(rgb_image)
        pil_image.save("debug_image.jpg")
        droneArmImage = self.currentImage.get('droneArmCamera', None)
        droneArmImage = cv2.cvtColor(droneArmImage, cv2.COLOR_BGR2RGB)
        pil_droneArmImage = PILImage.fromarray(droneArmImage)
        pil_droneArmImage.save("debug_droneArm_image.jpg")
        # sample = self.create_sample(pil_image)
        output = self.get_from_server(pil_image)
        print(output)
        
        self.write_to_pose(output)
    
    def convertTextToCommand(self, textAction):
        command={}
        command["type"] = textAction.get("type", "drone")
        command["activity"] = textAction.get("activity", "move")
        # if textAction["direction"]=="down":
        #     command["displacement"] = {"x": 0.0, "y": 0.0, "z": -0.05, "yaw": 0.0}
        # elif textAction["direction"]=="up":
        #     command["displacement"] = {"x": 0.0, "y": 0.0, "z": 0.05, "yaw": 0.0}
        if self.current_pose.pose.position.z < 0.5:
            disp = 0.02
        else:
            disp = 0.05
        if textAction["direction"]=="left":
            command["displacement"] = {"x": -disp, "y": 0.0, "z": 0.0, "yaw": 0.0}
        elif textAction["direction"]=="right":
            command["displacement"] = {"x": disp, "y": 0.0, "z": 0.0, "yaw": 0.0}
        elif textAction["direction"]=="above":
            command["displacement"] = {"x": 0.0, "y": disp, "z": 0.0, "yaw": 0.0}
        elif textAction["direction"]=="below":
            command["displacement"] = {"x": 0.0, "y": -disp, "z": 0.0, "yaw": 0.0}
        else:
            if textAction["centered"]==True:
                if self.current_pose.pose.position.z > 0.20:
                    command["displacement"] = {"x": 0.0, "y": 0.0, "z": -0.05, "yaw": 0.0}
                else:
                    command["displacement"] = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
                    command["activity"] = "close"
                    command["type"] = "gripper"
                    
            else:
                command["displacement"] = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}

        return command
    
    def wait(self, seconds):
        serverUrl = "http://192.168.209.86:8000/"
        endPoint= "dummyWait"
        headers = {
            "accept": "application/json",
            "Content-Type": "application/x-www-form-urlencoded"
        }
        try:
            data = {
                "seconds": seconds
            }
            response = requests.post(serverUrl+endPoint, data=data, headers=headers)
            return {}
        except Exception as e:
            self.get_logger().error(f"Error during request: {e}")
            return {}


    def get_from_server(self, image):
        isText = True
        serverUrl = "http://192.168.209.86:8000/"
        QwenendPoint= "predictQwen"
        LlavaendPoint= "predictLLava"
        try:
            response = requests.post(serverUrl+QwenendPoint, files={"image": open("debug_image.jpg", "rb"), "image2": open("debug_droneArm_image.jpg", "rb")})
            if response.status_code == 200:
                data = response.json()
                if isText:
                    data = self.convertTextToCommand(data)
                return data
            else:
                self.get_logger().error(f"Request failed with status code {response.status_code}: {response.text}")
                return {}
        except Exception as e:
            self.get_logger().error(f"Error during request: {e}")
            return {}
    

    def write_to_pose(self, msg):
        if msg['type'] == "gripper" and msg['activity'] == "close":
            self.grip_pose.pose.position.z = -60.0
            self.grip_publisher.publish(self.grip_pose)
            self.isGripClosed = True
            self.wait(5)
            return
        # Get the current time
        now = self.get_clock().now()
        self.current_pose.header.stamp = now.to_msg()
        self.current_pose.pose.position.x += msg['displacement']['x']
        self.current_pose.pose.position.y += msg['displacement']['y']
        self.current_pose.pose.position.z += (msg['displacement']['z'])
        # self.current_pose.pose.orientation.yaw += msg['displacement']['yaw']

        current_yaw = 2.0 * math.atan2(self.current_pose.pose.orientation.z, 
                                     self.current_pose.pose.orientation.w)
        self.current_pose.pose.orientation.z = math.sin(current_yaw / 2.0)
        self.current_pose.pose.orientation.w = math.cos(current_yaw / 2.0)
        self.pathTaken.append({
            "position": {
                "x": self.current_pose.pose.position.x,
                "y": self.current_pose.pose.position.y,
                "z": self.current_pose.pose.position.z
            },
            "orientation": {
                "x": self.current_pose.pose.orientation.x,
                "y": self.current_pose.pose.orientation.y,
                "z": self.current_pose.pose.orientation.z,
                "w": self.current_pose.pose.orientation.w
            },
            "timestamp": now.to_msg().sec
        })
        with open(self.filePath, 'w') as f:
            json.dump(self.pathTaken, f, indent=4)
        self.goal_publisher.publish(self.current_pose)
        self.get_logger().info(f"Published new goal pose: {self.current_pose.pose.position}\n")
        delay = 5.0 
        time.sleep(delay)
        # self.get_logger().info(f"Waiting for {delay:.2f} seconds before next update...")

            
    # def generate_text_from_sample(self,sample, max_new_tokens=1024, device="cuda"):
    #     self.model.to(device)
    #     # Prepare the text input by applying the chat template
    #     text_input = self.processor.apply_chat_template(
    #         sample['messages'][0:2],  # Use the sample without the system message
    #         tokenize=False,
    #         add_generation_prompt=True
    #     )

    #     # Process the visual input from the sample
    #     image_inputs, _ = process_vision_info(sample['messages'])

    #     # Prepare the inputs for the model
    #     model_inputs = self.processor(
    #         text=[text_input],
    #         images=image_inputs,
    #         return_tensors="pt",
    #     ).to(device)
    #     # .to(device)
    #     # .to(device)  # Move inputs to the specified device

    #     # Generate text with the model
    #     generated_ids = self.model.generate(**model_inputs, max_new_tokens=max_new_tokens)

    #     # Trim the generated ids to remove the input ids
    #     trimmed_generated_ids = [
    #         out_ids[len(in_ids):] for in_ids, out_ids in zip(model_inputs.input_ids, generated_ids)
    #     ]

    #     # Decode the output text
    #     output_text = self.processor.batch_decode(
    #         trimmed_generated_ids,
    #         skip_special_tokens=True,
    #         clean_up_tokenization_spaces=False
    #     )

    #     return output_text[0]  # Return the first decoded output text



def main(args=None):
    rclpy.init(args=args)
    joystick_controller = LlavaController()
    rclpy.spin(joystick_controller)
    joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
