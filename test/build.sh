#!/bin/bash
g++ -std=c++17 zmq_receiver.cpp -o zmq_receiver -lzmq -I../include
echo "Compiled: ./zmq_receiver"
