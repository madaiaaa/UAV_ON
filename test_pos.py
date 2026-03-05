import airsim
import json
import os
import subprocess
import time
import tempfile
from pathlib import Path

# 配置VLN
AIRSIM_SETTINGS_TEMPLATE = {
    "SeeDocsAt": "https://microsoft.github.io/AirSim/settings/",
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockSpeed": 1, # 仿真加速
    "Vehicles": {
        "Drone_1": {
            "VehicleType": "SimpleFlight",
            "X": 0, # 初始位置
            "Y": 0,
            "Z": 0,
            "Roll": 0,
            "Pitch": 0,
            "Yaw": 0,
            "Cameras": {
                    "0": {
                    "X": 1,
                    "Y": 0,
                    "Z": 0,
                    "Pitch": 0,
                    "Roll": 0,
                    "Yaw": 0,
                    "CaptureSettings": [
                        {
                        "ImageType": 0,
                        "Width": 512,
                        "Height": 512,
                        "FOV_Degrees": 90,
                        "AutoExposureMaxBrightness": 1,
                        "AutoExposureMinBrightness": 0.03
                        },
                        {
                        "ImageType": 2,
                        "Width": 256,
                        "Height": 256,
                        "FOV_Degrees": 90,
                        "AutoExposureMaxBrightness": 1,
                        "AutoExposureMinBrightness": 0.03
                        }
                    ]
                    },
                    "1": {
                    "X": 0,
                    "Y": -1,
                    "Z": 0,
                    "Pitch": 0,
                    "Roll": 0,
                    "Yaw": -90,
                    "CaptureSettings": [
                        {
                        "ImageType": 0,
                        "Width": 512,
                        "Height": 512,
                        "FOV_Degrees": 90,
                        "AutoExposureMaxBrightness": 1,
                        "AutoExposureMinBrightness": 0.03
                        },
                        {
                        "ImageType": 2,
                        "Width": 256,
                        "Height": 256,
                        "FOV_Degrees": 90,
                        "AutoExposureMaxBrightness": 1,
                        "AutoExposureMinBrightness": 0.03
                        }
                    ]
                    },
                    "2": {
                    "X": 0,
                    "Y": 1,
                    "Z": 0,
                    "Pitch": 0,
                    "Roll": 0,
                    "Yaw": 90,
                    "CaptureSettings": [
                        {
                        "ImageType": 0,
                        "Width": 512,
                        "Height": 512,
                        "FOV_Degrees": 90,
                        "AutoExposureMaxBrightness": 1,
                        "AutoExposureMinBrightness": 0.03
                        },
                        {
                        "ImageType": 2,
                        "Width": 256,
                        "Height": 256,
                        "FOV_Degrees": 90,
                        "AutoExposureMaxBrightness": 1,
                        "AutoExposureMinBrightness": 0.03
                        }
                    ]
                    },
                    "3": {
                    "X": 0,
                    "Y": 0,
                    "Z": 0,
                    "Pitch": -90,
                    "Roll": 0,
                    "Yaw": 0,
                    "CaptureSettings": [
                        {
                        "ImageType": 0,
                        "Width": 512,
                        "Height": 512,
                        "FOV_Degrees": 90,
                        "AutoExposureMaxBrightness": 1,
                        "AutoExposureMinBrightness": 0.03
                        },
                        {
                        "ImageType": 2,
                        "Width": 256,
                        "Height": 256,
                        "FOV_Degrees": 90,
                        "AutoExposureMaxBrightness": 1,
                        "AutoExposureMinBrightness": 0.03
                        }
                    ]
                    },
                },
            }
    }
}

def launch_airsim_with_settings(env_binary_path):
    if not os.path.exists(str(CWD_DIR / 'settings')):
        os.makedirs(str(CWD_DIR / 'settings'), exist_ok=True)
    with open(str(CWD_DIR / 'settings'/'settings.json'), 'w', encoding='utf-8') as dump_f:
        dump_f.write(json.dumps(AIRSIM_SETTINGS_TEMPLATE))

    # 启动环境
    cmd = "bash {} -NoSound -NoVSync  --settings={} -windowed".format(
        env_binary_path,
        str(CWD_DIR / 'settings'/'settings.json'),
    )
    # 后台启动环境
    process = subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL)
    return process, str(CWD_DIR / 'settings'/'settings.json')

def getPose_By_frame(FRAME_ID = 28):
    with open(TRAJECTORY_PATH, 'r') as f:
        lines = f.readlines()
        data = json.loads(lines[FRAME_ID])
        pos = data["sensors"]["state"]["position"]
        quat = data["sensors"]["state"]["quaternionr"]
    return pos,quat

def getTarget():
    with open(SETTING_PATH, 'r', encoding='utf-8') as f:
        data = json.load(f)

        target_pose = data.get('pose', [[]])[0]

    return target_pose


def run_task(select_bit=1):
    # select_bit=1 表示加载FRAME_ID

    # 加载环境
    proc, tmp_json = launch_airsim_with_settings(ENV_PATH)
    time.sleep(5)

    # 创建客户端
    client = airsim.MultirotorClient()
    try:
        client.confirmConnection()

        if select_bit:
            # 获取位姿
            pos, quat = getPose_By_frame(FRAME_ID)
        else:
            pos, quat = getTarget(), [0.0,0,0,0,0,0,0]
        
        print(f'posision: {pos}')
        print(f"quanternionr: {quat}")

        client.enableApiControl(True, vehicle_name="Drone_1")
        client.armDisarm(True)
        client.takeoffAsync()

        # 传送并悬空
        target_pose = airsim.Pose(
            position_val=airsim.Vector3r(x_val=pos[0], y_val=pos[1], z_val=pos[2]),
            orientation_val=airsim.Quaternionr(x_val=quat[0], y_val=quat[1], z_val=quat[2], w_val=quat[3])
        )
        client.simSetVehiclePose(target_pose, ignore_collision=True)
        
        vehicles = client.listVehicles()
        client.simSetObjectScale(vehicles[0],airsim.Vector3r(0.5,0.5,0.5))

        
        client.simContinueForFrames(50)
        client.simPause(True)


        os.makedirs("output_imgs", exist_ok=True)
        for cam_id, name in zip(["0", "1", "2", "3"], ["front", "left", "right", "bottom"]):
            try:
                resp = client.simGetImages([airsim.ImageRequest(cam_id, airsim.ImageType.Scene, pixels_as_float=False, compress=True)])[0]
                if select_bit:
                    airsim.write_file(f"output_imgs/frame_{FRAME_ID}_{name}.png", resp.image_data_uint8)
                else:
                    break
                print(f"保存成功: {name}")
            except Exception as e:
                print(f"相机 {cam_id} 获取失败: {e}")

    except Exception as e:
        print(f"运行出错: {e}")


if __name__ == '__main__':
    ENV_PATH = "../TEST_ENVS/Barnyard/Barnyard_test1.sh" 
    TRAJECTORY_PATH = "logs/scene/success_Barnyard.json/task_41/log/trajectory.jsonl"
    SETTING_PATH = "logs/scene/success_Barnyard.json/task_41/object_description.json"
    FRAME_ID = 26
    CWD_DIR = Path(str(os.path.abspath(__file__))).parent.resolve()
    run_task(1)