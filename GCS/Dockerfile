# Use a base Python image
FROM python:3.9-slim

# Set the working directory
WORKDIR /app

# Install system libraries for tkinter
RUN apt-get update && apt-get install -y \
    python3-tk \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set up the working directory
WORKDIR /GCS

# Copy project files into the container
COPY . /GCS

# Install dependencies
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir -r requirements.txt

# Set PYTHONPATH to include the project directory
ENV PYTHONPATH=/GCS

# Set the default command (optional)
CMD ["bash"]