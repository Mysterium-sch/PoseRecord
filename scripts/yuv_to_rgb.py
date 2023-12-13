# thanks Titon

import numpy as np
import cv2
import os

def yuv_420_888_to_rgb(arr, w, h):
    """Convert YUV_420_888 semi-planner image into an RGB image."""
    if arr.size % (w * h // 2) != 0:
        raise ValueError("Passed image size is incorrect!")

    arr = arr.reshape(((h + h // 2), w))
    uv_arr = arr[h:, :]

    # Recover Y plane.
    Y = arr[:h, :]

    # Recover Cr plane.
    Cr = np.zeros((h, w), dtype=np.float32)
    Cr[0::2, 0::2] = uv_arr[:, 1::2]
    Cr[0::2, 1::2] = uv_arr[:, 1::2]
    Cr[1::2, 0::2] = uv_arr[:, 1::2]
    Cr[1::2, 1::2] = uv_arr[:, 1::2]

    # Recover Cb plane.
    Cb = np.zeros((h, w), dtype=np.float32)
    Cb[0::2, 0::2] = uv_arr[:, 0::2]
    Cb[0::2, 1::2] = uv_arr[:, 0::2]
    Cb[1::2, 0::2] = uv_arr[:, 0::2]
    Cb[1::2, 1::2] = uv_arr[:, 0::2]

    # Formula for the YCbCr to RGB for android has been copied from:
    # https://en.wikipedia.org/wiki/YCbCr ; Section 21
    # -------------------------------------------------------------,
    delta = 128
    Cr = Cr - delta
    Cb = Cb - delta

    rgb = np.zeros((h, w, 3), dtype=np.float32)
    rgb[:, :, 0] = Y + 1.370705 * Cr
    rgb[:, :, 1] = Y - 0.698001 * Cr - 0.337633 * Cb
    rgb[:, :, 2] = Y + 1.732446 * Cb
    # -------------------------------------------------------------'

    return np.clip(rgb, a_min=0, a_max=255).astype(np.uint8)



def fix_yuv_buffer(arr, w, h):
    """Fix improperly recorded YUV buffer.

    Input buffer format:
    - Y-Plane (size: h x w)
    - U view of UV-Plane ( size: h/2 x w - 1)
    - V view of UV-Plane ( size: h/2 x w - 1)

    Output buffer format:
    - Y-Plane (size: h x w)
    - UV-Plane (size: h/2 x w)
    """
    y_size = w * h
    uv_size = w * h // 2
    fixed = np.zeros(y_size + uv_size, dtype=np.uint8)

    # Copy the Y-plane as is.
    fixed[:y_size] = arr[:y_size]

    # Copy the UV plane. This was originally copied as the
    # memory view of the U-plane. As such, the last byte of
    # the UV plane is missing.
    fixed[y_size:y_size + uv_size - 1] = arr[y_size: y_size + uv_size - 1]

    # Fix the last byte.
    fixed[y_size + uv_size - 1] = arr[-1]

    return fixed

IMAGE_W = 1920
IMAGE_H = 1080

def convert(yuv_path, rgb_path):
    with open(yuv_path, 'rb') as f:
        yuv_data = f.read()

    yuv_img = fix_yuv_buffer(
        np.frombuffer(yuv_data, dtype=np.uint8), IMAGE_W, IMAGE_H)
    rgb_img = yuv_420_888_to_rgb(yuv_img, IMAGE_W, IMAGE_H)
    bgr_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)
    rotated = cv2.rotate(bgr_img, cv2.ROTATE_90_CLOCKWISE)
    return rotated

if __name__ == "__main__":
    image_path = 'images'
    rgb_image_path = 'rgb_images'
    if not os.path.isdir(rgb_image_path):
        os.mkdir(rgb_image_path)
    num_of_images = len(os.listdir(image_path))
    for i in range(0, num_of_images):
        inpath = f"{image_path}/{i}.yuv"
        outpath = f"{rgb_image_path}/{i}.png"
        print (f'Converting image: {i}.yuv to {i}.png')
        convert(inpath, outpath)