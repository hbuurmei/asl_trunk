import cv2 
import numpy as np
import os
import random

def resize_image(image, target_size=(1080, 1080)):
    return cv2.resize(image, target_size)

def adjust_hue(image, hue_factor):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv_image[:, :, 0] = (hsv_image[:, :, 0] + hue_factor) % 180
    return cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

def adjust_value(image, value_factor):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv_image = np.array(hsv_image, dtype=np.float64)
    hsv_image[:, :, 2] = hsv_image[:, :, 2] * value_factor
    hsv_image[:, :, 2][hsv_image[:, :, 2] > 255] = 255
    hsv_image = np.array(hsv_image, dtype=np.uint8)
    return cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

def rotate_image(image, angle):
    if angle == 90:
        return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    elif angle == 180:
        return cv2.rotate(image, cv2.ROTATE_180)
    elif angle == 270:
        return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    else:
        return image

def adjust_bbox(bbox, image_shape, rotation_angle):
    height, width = image_shape[:2]
    c, x_center, y_center, w, h = bbox

    # Calculate new center based on rotation
    new_x_center, new_y_center = x_center * width, y_center * height
    if rotation_angle == 90:
        new_x_center, new_y_center = height - new_y_center, new_x_center
        temp = w
        w = h
        h = temp
    elif rotation_angle == 180:
        new_x_center, new_y_center = width - new_x_center, height - new_y_center
    elif rotation_angle == 270:
        new_x_center, new_y_center = new_y_center, width - new_x_center
        temp = w
        w = h
        h = temp

    # Convert back to normalized coordinates
    new_x_center /= height
    new_y_center /= width

    return [c, new_x_center, new_y_center, w, h]

def crop_image(image, left_pct, right_pct, top_pct, bottom_pct):
    height, width = image.shape[:2]

    # Calculate pixel values to crop
    left = int(width * left_pct)
    right = int(width * (1 - right_pct))
    top = int(height * top_pct)
    bottom = int(height * (1 - bottom_pct))

    # Crop the image using calculated pixel values
    cropped_image = image[top:bottom, left:right]

    return cropped_image

# use for val and test set
def crop_and_resize(image_path, output_dir):
    image = cv2.imread(image_path)
    base_name = os.path.basename(image_path)
    name, ext = os.path.splitext(base_name)
    # crop image to relevant region
    image = crop_image(image, left_pct=0.08, right_pct=0.05, top_pct=0, bottom_pct=0)
 
    # Resize image ( we want default to be 1080x1080)
    image = resize_image(image)
    cv2.imwrite(os.path.join(output_dir, f"{name}{ext}"), image)

# use for training set only
def augment_image(image_path, output_dir):
    image = cv2.imread(image_path)
    # print(image_path)
    base_name = os.path.basename(image_path)
    name, ext = os.path.splitext(base_name)
    # print(base_name, image.shape)

    # crop image to relevant region
    image = crop_image(image, left_pct=0.08, right_pct=0.05, top_pct=0, bottom_pct=0)
    # print(image.shape) #uncomment to see image size before scaling

    # Resize image ( we want default to be 1080x1080)
    image = resize_image(image)
    cv2.imwrite(os.path.join(output_dir, f"{name}{ext}"), image)

    # Random hue adjustment
    hue_factor = random.randint(-15, 15)
    hue_adjusted_image = adjust_hue(image, hue_factor)

    # Random value adjustment
    value_factor = random.uniform(0.5, 1.5)
    value_adjusted_image = adjust_value(hue_adjusted_image, value_factor)

    # Save augmented image and adjusted bounding boxes
    cv2.imwrite(os.path.join(output_dir, f"{name}_augmented{ext}"), value_adjusted_image)

    # return augmented filename
    return f"{name}_augmented{ext}"