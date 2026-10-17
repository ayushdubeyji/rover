#!/bin/bash
IMG="/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/rover_crawling.jpg"
OUT_MP4="/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/rover_crawl.mp4"
OUT_WEBM="/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/rover_crawl.webm"

ffmpeg -y -loop 1 -i "$IMG" -vf "zoompan=z='min(zoom+0.0012,1.2)':x='iw/2-(iw/zoom/2)+sin(in/20)*12':y='ih/2-(ih/zoom/2)+cos(in/25)*6':d=300:s=1280x720:fps=30" -t 10 -c:v libx264 -pix_fmt yuv420p "$OUT_MP4"
ffmpeg -y -i "$OUT_MP4" -c:v libvpx-vp9 -b:v 1M "$OUT_WEBM"
echo VIDEO_GENERATION_COMPLETE
