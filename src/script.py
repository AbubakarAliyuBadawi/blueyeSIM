from flask import Flask, Response, request
import cv2
import threading
import time
import logging
import json
import urllib.request
import ipaddress
import socket

app = Flask(__name__)
# Set up logging
logging.basicConfig(filename='visitor_log.txt', level=logging.INFO, 
                    format='%(asctime)s - %(message)s')

# Specifically target the C920 webcam at /dev/video4
camera = cv2.VideoCapture('/dev/video4')

# If that fails, try a different approach
if not camera.isOpened():
    print("Failed to open camera at /dev/video4, trying default camera...")
    camera = cv2.VideoCapture(0)

# Set better resolution for the C920 - it supports up to 1080p
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
camera.set(cv2.CAP_PROP_FPS, 30)

# Check if camera opened successfully
if not camera.isOpened():
    print("Error: Could not open webcam!")
    exit(1)
else:
    print("Successfully opened webcam")

output_frame = None
lock = threading.Lock()
public_ip = None
public_location = None

def is_private_ip(ip):
    """Check if an IP address is private"""
    try:
        return ipaddress.ip_address(ip).is_private
    except:
        return False

def get_public_ip():
    """Get your public IP address"""
    try:
        response = urllib.request.urlopen('https://api.ipify.org/')
        return response.read().decode('utf-8')
    except:
        return None

def get_geolocation(ip):
    """Use a free API to get geolocation information"""
    try:
        # Skip geolocation for private IPs
        if is_private_ip(ip):
            # For private IPs, use the server's public location but mark as local network
            global public_ip, public_location
            
            # If we haven't fetched the public IP yet, do it now
            if public_ip is None:
                public_ip = get_public_ip()
                
                if public_ip:
                    # Get the location of the public IP
                    url = f"https://ipapi.co/{public_ip}/json/"
                    response = urllib.request.urlopen(url)
                    data = json.loads(response.read().decode())
                    
                    if 'error' not in data:
                        public_location = {
                            'country': data.get('country_name', 'Unknown'),
                            'city': data.get('city', 'Unknown'),
                            'latitude': data.get('latitude', 0),
                            'longitude': data.get('longitude', 0)
                        }
            
            # Return the public location but mark as local network
            if public_location:
                local_location = public_location.copy()
                local_location['is_local'] = True
                local_location['local_ip'] = ip
                return local_location
            
            return None
        
        # For public IPs, get actual location
        url = f"https://ipapi.co/{ip}/json/"
        response = urllib.request.urlopen(url)
        data = json.loads(response.read().decode())
        
        if 'error' in data:
            return None
            
        location_info = {
            'country': data.get('country_name', 'Unknown'),
            'city': data.get('city', 'Unknown'),
            'latitude': data.get('latitude', 0),
            'longitude': data.get('longitude', 0),
            'is_local': False
        }
        return location_info
    except Exception as e:
        print(f"Geolocation error: {str(e)}")
        return None

def generate_frames():
    global output_frame, lock
    while True:
        success, frame = camera.read()
        if not success:
            print("Failed to read frame")
            time.sleep(0.1)
            continue
            
        # Acquire the lock
        with lock:
            output_frame = frame.copy()
            
        # Encode the frame as JPEG
        ret, buffer = cv2.imencode('.jpg', output_frame)
        frame = buffer.tobytes()
        
        # Yield the frame in the correct format for a multipart HTTP response
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    # Log the visitor's IP address
    visitor_ip = request.remote_addr
    
    # Try to get the real IP if behind a proxy
    if request.headers.get('X-Forwarded-For'):
        visitor_ip = request.headers.get('X-Forwarded-For').split(',')[0]
    
    # Get user agent for device info
    user_agent = request.headers.get('User-Agent', 'Unknown')
    
    # Get geolocation information
    geo_info = get_geolocation(visitor_ip)
    
    if geo_info:
        if geo_info.get('is_local', False):
            location_info = f"Local Network IP={geo_info['local_ip']} | Approximate Location: " \
                            f"Country: {geo_info['country']}, " \
                            f"City: {geo_info['city']}, " \
                            f"Lat: {geo_info['latitude']}, Long: {geo_info['longitude']}"
        else:
            location_info = f"Country: {geo_info['country']}, " \
                            f"City: {geo_info['city']}, " \
                            f"Lat: {geo_info['latitude']}, Long: {geo_info['longitude']}"
        
        logging.info(f"Visitor: IP={visitor_ip} | {location_info} | Device: {user_agent}")
    else:
        logging.info(f"Visitor: IP={visitor_ip} | Geolocation failed | Device: {user_agent}")
    
    return """
    <html>
    <head>
    <title>Badawi's C920 Webcam Stream</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            text-align: center;
            background-color: #f0f0f0;
        }
        h1 {
            color: #333;
        }
        .stream-container {
            margin: 20px auto;
            max-width: 1280px;
            border: 1px solid #ccc;
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
        }
        img {
            width: 100%;
            height: auto;
        }
    </style>
    </head>
    <body>
    <h1>Badawi's Logitech C920 Live Stream</h1>
    <div class="stream-container">
        <img src="/video_feed">
    </div>
    </body>
    </html>
    """

@app.route('/video_feed')
def video_feed():
    # Log access to the video feed specifically
    visitor_ip = request.remote_addr
    if request.headers.get('X-Forwarded-For'):
        visitor_ip = request.headers.get('X-Forwarded-For').split(',')[0]
    logging.info(f"Video feed accessed by IP: {visitor_ip}")
    
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/admin')
def admin_page():
    # A simple admin page to view visitor logs and show their locations on a map
    try:
        with open('visitor_log.txt', 'r') as f:
            logs = f.readlines()
        
        # Process the logs to extract visitor information
        visitors_data = []
        local_visitors = []
        
        for log in logs:
            if 'Lat:' in log and 'Long:' in log:
                try:
                    # Extract latitude and longitude
                    lat_start = log.find('Lat: ') + 5
                    lat_end = log.find(', Long:')
                    lng_start = log.find('Long: ') + 6
                    lng_end = log.find(' |', lng_start) if ' |' in log[lng_start:] else len(log)
                    
                    lat = float(log[lat_start:lat_end].strip())
                    lng = float(log[lng_start:lng_end].strip())
                    
                    # Extract timestamp
                    timestamp = log.split(' - ')[0].strip()
                    
                    # Extract IP
                    ip_start = log.find('IP=') + 3
                    ip_end = log.find(' |', ip_start)
                    ip = log[ip_start:ip_end].strip()
                    
                    # Check if this is a local network visitor
                    is_local = 'Local Network IP=' in log
                    local_ip = None
                    
                    if is_local:
                        local_ip_start = log.find('Local Network IP=') + 16
                        local_ip_end = log.find(' |', local_ip_start)
                        local_ip = log[local_ip_start:local_ip_end].strip()
                        
                        # Add to local visitors list
                        local_visitors.append({
                            'timestamp': timestamp,
                            'ip': local_ip,
                            'lat': lat,
                            'lng': lng
                        })
                    else:
                        # Add to regular visitors list
                        visitors_data.append({
                            'timestamp': timestamp,
                            'ip': ip,
                            'lat': lat,
                            'lng': lng
                        })
                except Exception as e:
                    print(f"Error parsing log: {str(e)}")
                    continue
        
        # Create map markers for all visitors
        map_markers = ""
        
        # Add regular visitors as blue markers
        for visitor in visitors_data:
            map_markers += f"""
                L.marker([{visitor['lat']}, {visitor['lng']}], {{
                    icon: L.icon({{
                        iconUrl: 'https://cdn.rawgit.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-blue.png',
                        shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
                        iconSize: [25, 41],
                        iconAnchor: [12, 41],
                        popupAnchor: [1, -34],
                        shadowSize: [41, 41]
                    }})
                }}).addTo(map).bindPopup('Public IP: {visitor['ip']} - {visitor['timestamp']}');
            """
        
        # Add local visitors as red markers
        for visitor in local_visitors:
            map_markers += f"""
                L.marker([{visitor['lat']}, {visitor['lng']}], {{
                    icon: L.icon({{
                        iconUrl: 'https://cdn.rawgit.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-red.png',
                        shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
                        iconSize: [25, 41],
                        iconAnchor: [12, 41],
                        popupAnchor: [1, -34],
                        shadowSize: [41, 41]
                    }})
                }}).addTo(map).bindPopup('Local Network IP: {visitor['ip']} - {visitor['timestamp']} <br><small>(Approximate location based on network)</small>');
            """
        
        # Format all logs for display
        log_html = '<br>'.join(logs[-50:])  # Show last 50 logs
        
        return f"""
        <html>
        <head>
            <title>Visitor Logs and Map</title>
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
            <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
            <style>
                body {{ font-family: monospace; padding: 20px; margin: 0; }}
                h1 {{ color: #333; }}
                pre {{ background: #f5f5f5; padding: 10px; border-radius: 5px; max-height: 300px; overflow: auto; }}
                #map {{ height: 400px; width: 100%; margin-bottom: 20px; }}
                .container {{ display: flex; flex-direction: column; padding: 20px; }}
                .legend {{ background: white; padding: 10px; border-radius: 5px; }}
                .legend-item {{ display: flex; align-items: center; margin-bottom: 5px; }}
                .legend-color {{ width: 20px; height: 20px; margin-right: 5px; border-radius: 50%; }}
                .blue {{ background-color: blue; }}
                .red {{ background-color: red; }}
            </style>
        </head>
        <body>
            <div class="container">
                <h1>Visitor Locations</h1>
                <div id="map"></div>
                
                <div class="legend">
                    <div class="legend-item">
                        <div class="legend-color blue"></div>
                        <span>Public IP visitors</span>
                    </div>
                    <div class="legend-item">
                        <div class="legend-color red"></div>
                        <span>Local network visitors (approximate location)</span>
                    </div>
                </div>
                
                <h1>Recent Logs</h1>
                <pre>{log_html}</pre>
            </div>
            
            <script>
                var map = L.map('map').setView([20, 0], 2);
                L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                }}).addTo(map);
                
                // Add markers for all visitors
                {map_markers}
                
                // If we have visitors, zoom to fit them
                var visitorCount = {len(visitors_data) + len(local_visitors)};
                if (visitorCount > 0) {{
                    // Resize map after a slight delay to ensure it's properly initialized
                    setTimeout(function() {{
                        map.invalidateSize();
                    }}, 100);
                }}
            </script>
        </body>
        </html>
        """
    except Exception as e:
        return f"Error displaying logs: {str(e)}"

if __name__ == '__main__':
    try:
        # Get the local IP address to display
        import socket
        hostname = socket.gethostname()
        local_ip = socket.gethostbyname(hostname)
        print(f"Starting server on http://{local_ip}:5000")
        print(f"Admin page available at http://{local_ip}:5000/admin")
        
        app.run(host='0.0.0.0', port=5000, debug=False)  # Set debug to False to avoid reloading issues
    finally:
        # Make sure to release the camera when the program exits
        if camera is not None:
            camera.release()
            print("Camera released")