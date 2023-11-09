import numpy as np
import os
from tensorflow.keras.preprocessing.image import load_img, img_to_array
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, UpSampling2D
import matplotlib.pyplot as plt
from tensorflow.keras.layers import Reshape
from tensorflow.keras.preprocessing.image import array_to_img
from tensorflow.keras.layers import Cropping2D


MASK_IMG_DIR = "C:/Users/apant/OneDrive/Desktop/Capstone/MaskImages/"
RAW_IMG_DIR = "C:/Users/apant/OneDrive/Desktop/Capstone/RawImages/"
PDCT_IMG_DIR ="C:/Users/apant/OneDrive/Desktop/Capstone/Pdct_Images/"

IMG_SIZE = (300, 800)

# 1. Load and preprocess the data
def load_images_from_folder(folder, target_size):
    images = []
    for filename in os.listdir(folder):
        img = load_img(os.path.join(folder, filename), target_size=target_size, color_mode='grayscale')
        img = img_to_array(img) / 255.0
        images.append(img)
    return np.array(images)

raw_images = load_images_from_folder(RAW_IMG_DIR, IMG_SIZE)
mask_images = load_images_from_folder(MASK_IMG_DIR, IMG_SIZE)


model = Sequential()

# Downsampling (Encoder)
model.add(Conv2D(32, (3, 3), activation='relu', padding='same', input_shape=(IMG_SIZE[0], IMG_SIZE[1], 1)))
model.add(MaxPooling2D((2, 2), padding='same'))
model.add(Conv2D(64, (3, 3), activation='relu', padding='same'))
model.add(MaxPooling2D((2, 2), padding='same'))
model.add(Conv2D(128, (3, 3), activation='relu', padding='same'))
model.add(MaxPooling2D((2, 2), padding='same'))
model.add(Conv2D(256, (3, 3), activation='relu', padding='same'))
model.add(MaxPooling2D((2, 2), padding='same'))

# Bottleneck
model.add(Conv2D(512, (3, 3), activation='relu', padding='same'))

# Upsampling (Decoder)
model.add(UpSampling2D((2, 2)))
model.add(Conv2D(256, (3, 3), activation='relu', padding='same'))
model.add(UpSampling2D((2, 2)))
model.add(Conv2D(128, (3, 3), activation='relu', padding='same'))
model.add(UpSampling2D((2, 2)))
model.add(Conv2D(64, (3, 3), activation='relu', padding='same'))
model.add(UpSampling2D((2, 2)))
model.add(Conv2D(32, (3, 3), activation='relu', padding='same'))
model.add(Cropping2D(cropping=((2, 2), (0, 0))))

model.add(Conv2D(1, (3, 3), activation='sigmoid', padding='same'))

model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

# 3. Train the model
model.fit(raw_images, mask_images, epochs=50, batch_size=32, validation_split=0.2)

# 4. Save the model for future use
model.save('mask_predictor.h5')

def predict_mask(raw_image_path, model, target_size):
    img = load_img(raw_image_path, target_size=target_size, color_mode='grayscale')
    img_array = img_to_array(img) / 255.0
    img_array_expanded = np.expand_dims(img_array, axis=0)
    
    predicted_mask = model.predict(img_array_expanded)
    
    return img_array, predicted_mask[0]

def display_prediction(raw_image_path, model, imgName, target_size=IMG_SIZE):
    raw_image_path = raw_image_path + imgName
    raw_image, predicted_mask = predict_mask(raw_image_path, model, target_size)
    
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.imshow(raw_image.squeeze(), cmap='gray')
    plt.title('Raw Image')
    
    plt.subplot(1, 2, 2)
    plt.imshow(predicted_mask.squeeze(), cmap='gray')
    plt.title('Predicted Mask')

    plt.show()

def save_predicted_mask(raw_image_path, model, save_path, target_size=IMG_SIZE):
    raw_image, predicted_mask = predict_mask(raw_image_path, model, target_size)
    
    # Convert array to image and save
    mask_img = array_to_img(predicted_mask)
    mask_img.save(save_path)

def is_image_file(filename):
    valid_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif']
    return any(filename.lower().endswith(ext) for ext in valid_extensions)

# Process all images in RAW_IMG_DIR and save predictions to PDCT_IMG_DIR
for filename in os.listdir(RAW_IMG_DIR):
    if is_image_file(filename):  # Check if the file is an image
        raw_image_path = os.path.join(RAW_IMG_DIR, filename)
        save_path = os.path.join(PDCT_IMG_DIR, filename)
        
        if os.path.exists(save_path):
            print(f"Warning: Overwriting existing file: {save_path}")
        
        save_predicted_mask(raw_image_path, model, save_path)
