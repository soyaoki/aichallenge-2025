pip install uv
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

sudo rm -rf /autoware/install/autoware_perception_msgs/
sudo rm -rf /autoware/install/autoware_planning_msgs/

sudo apt update -y
# sudo apt dist-upgrade -y
sudo apt install ros-humble-autoware-common-msgs -y
sudo apt install ros-humble-autoware-perception-msgs -y
sudo apt install ros-humble-autoware-planning-msgs -y
sudo apt install ros-humble-autoware-internal-perception-msgs -y
sudo apt install ros-humble-autoware-internal-planning-msgs -y
sudo apt install ros-humble-autoware-internal-msgs -y

cd /aichallenge/workspace/src/aichallenge_submit/vlm_trajectory_selector
$HOME/.local/bin/uv venv -p python3.10
source .venv/bin/activate
$HOME/.local/bin/uv pip install .
