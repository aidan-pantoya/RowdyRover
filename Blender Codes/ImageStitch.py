from PIL import Image
import sys
import os

def concatenate_images(img1_path, img2_path, output_path):
    # Load the images
    img1 = Image.open(img1_path)
    img2 = Image.open(img2_path)
    
    # Concatenate horizontally
    concatenated_img = Image.new('RGB', (img1.width + img2.width, img1.height))
    concatenated_img.paste(img1, (0,0))
    concatenated_img.paste(img2, (img1.width,0))
    
    concatenated_img.save(output_path)
