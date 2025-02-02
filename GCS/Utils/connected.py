import socket

def is_connected(host="8.8.8.8", port=53, timeout=3):
    """
    Check if the computer is connected to the internet.
    
    Args:
        host (str): A reliable external server to test connectivity (default: Google's public DNS).
        port (int): The port number to use (default: 53, DNS service).
        timeout (int): Connection timeout in seconds (default: 3).
    
    Returns:
        bool: True if connected, False otherwise.
    """
    try:
        socket.setdefaulttimeout(timeout)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
        return True
    except (socket.timeout, socket.error):
        return False