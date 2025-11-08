pip install uv
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

cd /aichallenge/workspace/src/aichallenge_submit/vlm_planner
$HOME/.local/bin/uv venv -p python3.10
source .venv/bin/activate
$HOME/.local/bin/uv pip install .
