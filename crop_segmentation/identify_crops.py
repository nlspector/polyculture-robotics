from inference_sdk import InferenceHTTPClient

CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="UKvqpSd8oBGio0MaWkZZ"
)

result = CLIENT.infer("photo1.jpeg", model_id="detect-pumpkin/1")
print(result)