FROM gocv/opencv:4.11.0-static

RUN apt-get update && apt-get install --no-install-recommends -y \
    libegl1 \
    libgl1 \
    libgomp1 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*


RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir --upgrade open3d opencv-python

WORKDIR /home
COPY ./src /home/src
    
CMD ["python3", "src/main.py"]