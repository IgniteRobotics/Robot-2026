from inference import InferencePipeline
from inference.core.interfaces.stream.sinks import render_boxes


api_key = "SEcM2IbWCtqkluULWHcQ"


pipeline = InferencePipeline.init(
    model_id = "pheonix-2026",
    video_reference = photon(0), # whatever id is given to the camera
    on_prediction = render_box,
    api_key = api_key,
)


pipeline.start()
pipeline.join()
