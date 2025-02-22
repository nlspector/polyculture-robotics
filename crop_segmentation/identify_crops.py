from inference_sdk import InferenceHTTPClient

import torch
import numpy as np
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

from PIL import Image
import time

start = time.time()
curr_ckpt = time.time()

device = torch.device("cpu")

sam2_checkpoint = "./checkpoints/sam2.1_hiera_small.pt"
model_cfg = "configs/sam2.1/sam2.1_hiera_s.yaml"

print("Building SAM2 model...")
sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=device)

print("Creating predictor...")
predictor = SAM2ImagePredictor(sam2_model)

print("Elapsed time: " + str(time.time() - curr_ckpt) + " (total elapsed: " + str(time.time() - start) + ")")
curr_ckpt = time.time()

print("Calling YOLO inference")
CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="UKvqpSd8oBGio0MaWkZZ"
)

result = CLIENT.infer("photo1.jpeg", model_id="detect-pumpkin/1")
print(result)
print("Elapsed time: " + str(time.time() - curr_ckpt) + " (total elapsed: " + str(time.time() - start) + ")")
curr_ckpt = time.time()

width_delta = result["predictions"][0]["width"] / 2
height_delta = result["predictions"][0]["height"] / 2

x1 = result["predictions"][0]["x"] - width_delta
y1 = result["predictions"][0]["y"] - height_delta
x2 = result["predictions"][0]["x"] + width_delta
y2 = result["predictions"][0]["y"] + height_delta

###### HELPER FUNCTIONS FROM COLAB ######

import matplotlib.pyplot as plt

np.random.seed(3)

def show_mask(mask, ax, random_color=False, borders = True):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30/255, 144/255, 255/255, 0.6])
    h, w = mask.shape[-2:]
    mask = mask.astype(np.uint8)
    mask_image =  mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    if borders:
        import cv2
        contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        # Try to smooth contours
        contours = [cv2.approxPolyDP(contour, epsilon=0.01, closed=True) for contour in contours]
        mask_image = cv2.drawContours(mask_image, contours, -1, (1, 1, 1, 0.5), thickness=2) 
    ax.imshow(mask_image)

def show_points(coords, labels, ax, marker_size=375):
    pos_points = coords[labels==1]
    neg_points = coords[labels==0]
    ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)
    ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)   

def show_box(box, ax):
    x0, y0 = box[0], box[1]
    w, h = box[2] - box[0], box[3] - box[1]
    ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor='green', facecolor=(0, 0, 0, 0), lw=2))    

def show_masks(image, masks, scores, point_coords=None, box_coords=None, input_labels=None, borders=True):
    for i, (mask, score) in enumerate(zip(masks, scores)):
        plt.figure(figsize=(10, 10))
        plt.imshow(image)
        show_mask(mask, plt.gca(), borders=borders)
        if point_coords is not None:
            assert input_labels is not None
            show_points(point_coords, input_labels, plt.gca())
        if box_coords is not None:
            # boxes
            show_box(box_coords, plt.gca())
        if len(scores) > 1:
            plt.title(f"Mask {i+1}, Score: {score:.3f}", fontsize=18)
        plt.axis('off')
        plt.show()

###### END HELPER FUNCTIONS ######


image = Image.open('photo1.jpeg')
image = np.array(image.convert("RGB"))

print("Setting image for SAM2...")
predictor.set_image(image)

input_box = np.array([int(x1), int(y1), int(x2), int(y2)])
print("Elapsed time: " + str(time.time() - curr_ckpt) + " (total elapsed: " + str(time.time() - start) + ")")
curr_ckpt = time.time()

print("SAM2: Predicting masks")
masks, scores, _ = predictor.predict(
    point_coords=None,
    point_labels=None,
    box=input_box[None, :],
    multimask_output=False,
)
print("Elapsed time: " + str(time.time() - curr_ckpt) + " (total elapsed: " + str(time.time() - start) + ")")
curr_ckpt = time.time()

show_masks(image, masks, scores, box_coords=input_box)