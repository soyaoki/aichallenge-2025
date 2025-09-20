sudo apt update
cd /aichallenge/workspace/src/aichallenge_submit/yabloc
rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro $ROS_DISTRO

mkdir -p ~/autoware_data/yabloc_pose_initializer/
wget -P ~/autoware_data/yabloc_pose_initializer/ \
       https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/136_road-segmentation-adas-0001/resources.tar.gz
tar xzf ~/autoware_data/yabloc_pose_initializer/resources.tar.gz -C ~/autoware_data/yabloc_pose_initializer/

sudo apt install python3-tomli -y