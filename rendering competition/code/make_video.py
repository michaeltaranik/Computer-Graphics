import cv2
import os

image_folder = 'renders'
video_name = 'video.mp4'

images = [img for img in os.listdir(image_folder) if img.endswith(".ppm")]
images.sort() # Ensure they are in 000, 001, 002 order

boomerang = images + images[::-1]
num_repeats = 8
images = num_repeats * boomerang
# Read first image to get dimensions
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'mp4v'), 24, (width, height))

for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()
print("Video created successfully!")