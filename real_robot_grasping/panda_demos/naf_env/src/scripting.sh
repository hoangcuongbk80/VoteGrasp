#!/bin/bash
clear
python main.py --num_episodes 50000 --batch_size 32
python main.py --num_episodes 50000 --batch_size 512
python main.py --num_episodes 50000 --batch_size 256
