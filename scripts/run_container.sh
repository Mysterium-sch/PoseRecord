#!/bin/bash
docker build -t bag_stuff .
docker run -v $(pwd):/app -w /app bag_stuff python3 data2bag.py