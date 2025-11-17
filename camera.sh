ros2 launch v4l2_camera v4l2_camera.launch.py \
  camera_name:=front \
  image_topic:=image_raw \
  v4l2_camera_param_path:=/home/sws/.ros/camera_info/camera_params.yaml \
  rate_diagnostics_param_path:=/home/sws/.ros/camera_info/rate_diagnostics.yaml \
  camera_info_url:=file:///home/sws/.ros/camera_info/gmsl2-usb3.0_conversion_kit.yaml \
  hardware_id:='/dev/video2' \
  use_sensor_data_qos:=True
