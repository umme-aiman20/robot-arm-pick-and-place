 🤖 Robotic Arm Pick and Place Simulation

A complete simulation of a **robotic arm** trained using **Reinforcement Learning (PPO)** in **PyBullet**, capable of performing autonomous **pick-and-place tasks** — with visual explanation using **Grad-CAM**.

---
 🧠 Key Features

- 🔧 Simulated realistic robotic arm environment using PyBullet
- 🤖 Trained using **Proximal Policy Optimization (PPO)** from Stable Baselines3
- 🎯 Pick-and-place task with **randomized object locations** for better generalization
- 🎓 Learned grasping through **reward shaping**
- 🔥 Visualized decision-making using **Grad-CAM heatmaps** over input images

---
📁 Project Structure

```bash
robot_arm/
├── env/
│   └── robot_env.py           # Custom Gym environment
├── train.py                   # Training script using PPO
├── evaluate.py                # Evaluates trained model
├── evaluate_with_gradcam.py   # Grad-CAM explanation
├── robot_arm_model.zip        # Trained model
├── gradcam_step_*.png         # Heatmap images
├── requirements.txt
└── README.md

🚀 Getting Started :

1️⃣ Clone this repo:
git clone https://github.com/umme-aiman20/robot-arm-pick-and-place.git
cd robot-arm-pick-and-place

2️⃣ Install dependencies:
pip install -r requirements.txt

3️⃣ Run simulation with Grad-CAM:
python evaluate_with_gradcam.py


🧠 How Grad-CAM Works Here
Grad-CAM highlights image regions that the RL policy focuses on when choosing actions.
You’ll see colorful heatmaps overlayed on camera views, showing the model’s visual attention.

📜 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


🙋‍♀️ Author:  
. 👩🏻‍💻 @umme-aiman20




