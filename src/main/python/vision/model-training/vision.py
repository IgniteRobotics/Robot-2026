import cv2
from inference_sdk import InferenceHTTPClient
from inference_sdk.webrtc import WebcamSource, StreamConfig, VideoMetadata
import math

# Initialize client
client = InferenceHTTPClient.init(
    api_url="https://serverless.roboflow.com",
    api_key="4VKcVzUhuwU1FXRNK0Y2"
)

# Configure video source (webcam)
source = WebcamSource(resolution=(1280, 720))

# Configure streaming options
config = StreamConfig(
    stream_output=["output_image"],  # Get video back with annotations
    data_output=["count_objects","predictions"]      # Get prediction data via datachannel,
    processing_timeout=3600              # 60 minutes,
    requested_plan="webrtc-gpu-medium",  # Options: webrtc-gpu-small, webrtc-gpu-medium, webrtc-gpu-large
    requested_region="us"                # Options: us, eu, ap
)

# Create streaming session
session = client.webrtc.stream(
    source=source,
    workflow="detect-count-and-visualize-2",
    workspace="pheonix-azogm",
    image_input="image",
    config=config
)

# Handle incoming video frames
@session.on_frame
def show_frame(frame, metadata):
    cv2.imshow("Workflow Output", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        session.close()

# Handle prediction data via datachannel
@session.on_data()
def on_data(data: dict, metadata: VideoMetadata):
    print(f"Frame {metadata.frame_id}: {data}")

# Run the session (blocks until closed)
session.run()

def closest_ball(predictions):
    lemons = [p for p in predictions id p.get('class') == 'lemon' or p.get('class_name') == 'lemon']
    if len(lemon) < 1:
        return None, None, float('inf') # going for a single ball wouldnt be efficient
    min_dist = float('inf')
    closest_set = (None, None)

    for i in range(len(lemon)):
        for j in range(i + 1, len(lemon)):
            p1, p2 = lemon[i], lemon[j]
            # Euclidean distance
            dist = math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_pair = (p1, p2)
                
    return closest_pair[0], closest_pair[1], min_dist