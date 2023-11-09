cd /home/rogat/ESP32-ROTEM-SELA/rotem_sela_top/backend-docker/rotem_sela_backend/
source .venv/bin/activate
echo "Hello"> allout.txt 2>&1
python new_video_main.py > allout.txt 2>&1
