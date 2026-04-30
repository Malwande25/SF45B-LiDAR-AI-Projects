LiDAR-Based Perception and Machine Learning for Autonomous Obstacle Detection (SF45/B Dataset)

This repository contains a series of AI & Machine Learning projects using the **LightWare SF45/B LiDAR sensor**.  
The goal is to explore **data collection, processing, ML classification, sensor fusion, anomaly detection, and reinforcement learning** for robotics and embedded AI applications.  

---

📂 Project Structure
- **01-data-collection-visualization** → Collect and visualize LiDAR scans.  
- **02-obstacle-detection-ml** → Classify safe vs blocked paths using ML.  
- **03-denoising-autoencoder** → Use deep learning to denoise LiDAR scans.  
- **04-sensor-fusion-lidar-camera** → Combine LiDAR + camera data with CNNs.  
- **05-trajectory-prediction** → LSTM-based prediction of moving obstacle paths.  
- **06-anomaly-detection** → Detect unexpected objects using unsupervised ML.  
- **07-shape-classification-pointnet** → Recognize 3D object shapes.  
- **08-reinforcement-learning-path-planning** → RL-based navigation using LiDAR inputs.  
- **09-predictive-maintenance** → ML for monitoring LiDAR health/performance.  

 📊 Current Progress
- [x] Setup repository  
- [x] Data collection & visualization  
- [ ] Obstacle detection with ML  
- [ ] Denoising with autoencoders  
- [ ] Sensor fusion (LiDAR + camera)  
- [ ] Trajectory prediction with LSTM  
- [ ] Anomaly detection  
- [ ] Shape classification with PointNet  
- [ ] RL for path planning  
- [ ] Predictive maintenance  

---

 🛠 Requirements
- Python 3.8+  
- ROS (for data collection & simulation)  
- TensorFlow / PyTorch  
- scikit-learn  
- matplotlib, pandas, numpy  

## Install dependencies:
```bash
pip install -r requirements.txt



