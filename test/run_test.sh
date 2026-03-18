#!/bin/bash
cd /home/zsc/Project/Cpp_Backend_3.12_gpu/test

echo "Starting ZMQ receiver..."
./zmq_receiver &
RECEIVER_PID=$!

sleep 2

echo "Starting vision system..."
cd /home/zsc/Project/Cpp_Backend_3.12_gpu
timeout 8 ./build/vision_test config.json

echo "Stopping receiver..."
kill $RECEIVER_PID 2>/dev/null

echo "Test completed"
